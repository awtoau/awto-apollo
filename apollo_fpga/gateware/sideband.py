#
# This file is part of Apollo.
#
# SPDX-License-Identifier: BSD-3-Clause

""" FPGA_ADV command responder.

Half-duplex request/response over the single FPGA_ADV wire, with Apollo as the
master. See docs/apollo_samd11_mcu/fpga-adv-command-protocol.md.

    Apollo -> FPGA:  [CMD]
    FPGA  -> Apollo: [STATUS][payload ...][CRC8]

Response lengths are fixed per command and known to both sides at compile time,
so there is no length field. That is what makes this module stateless between
commands: it receives one byte, transmits a complete response, and returns to
idle. There is no half-parsed condition to abandon, and therefore no timeout on
this side of the link.

The FPGA never transmits unasked, which is what makes the link collision-free
without arbitration -- only one side can be driving at any moment, by
construction rather than by protocol.
"""

from amaranth import Cat, Const, Elaboratable, Module, Mux, Signal, Array
from amaranth_stdio.serial import AsyncSerialRX


# Command opcodes. Must match the table in the protocol document.
CMD_PING   = 0x01
CMD_STATUS = 0x02
CMD_POWER  = 0x2B

# Response payload sizes, excluding the status byte and the CRC.
PAYLOAD_SIZE = {
    CMD_PING:   2,
    CMD_STATUS: 0,
    CMD_POWER:  16,
}

# Status byte bits.
STATUS_OK          = 0
STATUS_EVENTS      = 1
STATUS_ERROR       = 2
STATUS_RECONFIG    = 3
STATUS_STATE_SHIFT = 4
STATUS_HEARTBEAT   = 6

PROTOCOL_VERSION = 0x01


class CRC8(Elaboratable):
    """ CRC-8/ATM: polynomial 0x07, init 0x00, no reflection, no final XOR.

    One byte at a time, eight shifts per clock. At 115200 baud a byte takes
    ~87 us, so an eight-deep combinational chain is never the critical path.
    """

    def __init__(self):
        self.data   = Signal(8)
        self.strobe = Signal()
        self.clear  = Signal()
        self.crc    = Signal(8)

    def elaborate(self, platform):
        m = Module()

        with m.If(self.clear):
            m.d.sync += self.crc.eq(0)
        with m.Elif(self.strobe):
            value = self.crc ^ self.data
            for _ in range(8):
                value = Mux(value[7], (value << 1)[:8] ^ 0x07, (value << 1)[:8])
            m.d.sync += self.crc.eq(value)

        return m


class UARTRx(Elaboratable):
    """ 8-N-1 receiver.

    A thin adapter over amaranth-stdio's AsyncSerialRX rather than a
    hand-rolled state machine. Writing one from scratch here produced a
    receiver that framed correctly but decoded every byte as zero, and the
    upstream implementation is already tested against exactly this.
    """

    def __init__(self, divisor):
        self.divisor = divisor
        self.rx      = Signal(init=1)
        self.data    = Signal(8)
        self.strobe  = Signal()

    def elaborate(self, platform):
        m = Module()

        m.submodules.rx = rx = AsyncSerialRX(divisor=self.divisor)

        m.d.comb += [
            rx.i.eq(self.rx),
            # Always ready to accept: this responder acts on each byte
            # immediately, so there is nothing to back-pressure.
            rx.ack.eq(1),
            self.data.eq(rx.data),
            # rdy is a level; strobe once per byte so downstream logic that
            # counts bytes does not see one arrival as several.
            self.strobe.eq(rx.rdy & rx.ack),
        ]

        return m


class UARTTx(Elaboratable):
    """ 8-N-1 transmitter. Idles high, as a UART line at rest must. """

    def __init__(self, divisor):
        self.divisor = divisor
        self.data    = Signal(8)
        self.start   = Signal()
        self.tx      = Signal(init=1)
        self.busy    = Signal()

    def elaborate(self, platform):
        m = Module()

        counter = Signal(range(self.divisor))
        tick    = Signal()
        m.d.comb += tick.eq(counter == 0)
        m.d.sync += counter.eq(Mux(tick, self.divisor - 1, counter - 1))

        # start + 8 data + stop, shifted out LSB first; all-ones is idle.
        shifter = Signal(10, init=0x3FF)
        bit_no  = Signal(range(10))

        m.d.comb += self.tx.eq(shifter[0])

        with m.FSM():

            with m.State("IDLE"):
                m.d.comb += self.busy.eq(0)
                m.d.sync += shifter.eq(0x3FF)
                with m.If(self.start):
                    m.next = "LOAD"

            # Load on a bit boundary. Loading mid-bit would overwrite the
            # shifter before the previous stop bit had been held for its full
            # period, truncating one byte and corrupting the next.
            with m.State("LOAD"):
                m.d.comb += self.busy.eq(1)
                with m.If(tick):
                    m.d.sync += [
                        shifter.eq(Cat(Const(0, 1), self.data, Const(1, 1))),
                        bit_no.eq(0),
                    ]
                    m.next = "SHIFT"

            with m.State("SHIFT"):
                m.d.comb += self.busy.eq(1)
                with m.If(tick):
                    m.d.sync += [
                        shifter.eq(Cat(shifter[1:], 1)),
                        bit_no.eq(bit_no + 1),
                    ]
                    with m.If(bit_no == 9):
                        m.next = "IDLE"

        return m


class SidebandResponder(Elaboratable):
    """ Answers Apollo's commands on FPGA_ADV.

    I/O ports:
        I: rx           -- the FPGA_ADV line, as an input
        O: tx           -- what to drive on FPGA_ADV
        O: tx_active    -- high while transmitting, for tri-state control
        I: power_data   -- 128 bits: VBUS[0..3] then VSENSE[0..3], LE16 each
        I: events       -- events pending
        I: error        -- error flag
        I: reconfigured -- FPGA reconfigured since last poll
        I: state        -- 2-bit FPGA state
    """

    def __init__(self, clk_freq_hz=60e6, baud=115200):
        self.divisor = int(clk_freq_hz // baud)

        self.rx           = Signal(init=1)
        self.tx           = Signal(init=1)
        self.tx_active    = Signal()

        self.power_data   = Signal(128)
        self.events       = Signal()
        self.error        = Signal()
        self.reconfigured = Signal()
        self.state        = Signal(2)

        # Six-bit status display, for wiring to the board LEDs. A link that
        # fails silently is hard to diagnose; this makes the responder's state
        # visible without a debug session.
        #
        #   0  a command byte was received (stretched, so it is visible)
        #   1  currently transmitting a response
        #   2  last command was understood
        #   3  last command was POWER
        #   4  heartbeat -- toggles per response, so it blinks under polling
        #   5  a command was rejected as unknown (latched until the next good one)
        self.leds = Signal(6)

        # Raw receive taps, for instrumentation. Exposed so a test harness can
        # tell "nothing arrived" from "something arrived that was not a valid
        # command" -- the responder's own state only reflects bytes it accepted.
        self.rx_strobe = Signal()
        self.rx_byte   = Signal(8)

    def elaborate(self, platform):
        m = Module()

        m.submodules.rx_uart = rx_uart = UARTRx(self.divisor)
        m.submodules.tx_uart = tx_uart = UARTTx(self.divisor)
        m.submodules.crc     = crc     = CRC8()

        # Hold the receiver's input idle-high while we transmit.
        #
        # The pin is bidirectional, so pad.i reads back whatever pad.o drives:
        # without this the responder frames its own reply as incoming bytes and
        # dispatches on them, triggering further spurious replies. Observed as
        # three received bytes for a one-byte command, the last being our own
        # CRC.
        #
        # Idle-high rather than gating the strobe, so the receiver sees a clean
        # line and cannot be left mid-frame when transmission ends.
        m.d.comb += rx_uart.rx.eq(Mux(self.tx_active, 1, self.rx))
        m.d.comb += [
            self.rx_strobe.eq(rx_uart.strobe),
            self.rx_byte.eq(rx_uart.data),
        ]
        m.d.comb += self.tx.eq(tx_uart.tx)

        # Flips on every response: a value that *changes* proves the responder
        # is executing, where a repeated value could be a wedged state machine
        # replaying a stale buffer.
        heartbeat = Signal()

        #
        # Status LEDs.
        #
        # A received byte lasts one clock, which is invisible. Stretch it to
        # roughly 1/8 second so a single command produces a perceptible flash
        # and a polled link looks like steady flicker.
        #
        stretch_bits = max((self.divisor * 8).bit_length(), 20)
        rx_stretch   = Signal(stretch_bits)
        unknown_cmd  = Signal()
        last_power   = Signal()
        last_ok      = Signal()

        with m.If(rx_uart.strobe):
            m.d.sync += rx_stretch.eq(-1)
        with m.Elif(rx_stretch != 0):
            m.d.sync += rx_stretch.eq(rx_stretch - 1)

        status = Signal(8)
        ok     = Signal()
        m.d.comb += status.eq(Cat(
            ok,                 # bit 0
            self.events,        # bit 1
            self.error,         # bit 2
            self.reconfigured,  # bit 3
            self.state,         # bits 4-5
            heartbeat,          # bit 6
            Const(0, 1),        # bit 7
        ))

        # Payload bytes for the command in flight. Sized for the largest
        # response so one register file serves all commands.
        turnaround  = Signal(range(self.divisor * 2 + 1))
        payload_len = Signal(range(17))
        byte_index  = Signal(range(19))
        send_byte   = Signal(8)

        # PING's payload; POWER's comes from power_data.
        ping_payload = Array([Const(PROTOCOL_VERSION, 8), Const(0x00, 8)])

        # send_byte is registered, not combinational: UARTTx latches .data in
        # its LOAD state, one cycle after .start is asserted. A combinational
        # value would have returned to its default by then, which transmitted
        # every byte as zero.
        m.d.comb += [
            tx_uart.data.eq(send_byte),
            tx_uart.start.eq(0),
            crc.strobe.eq(0),
            crc.clear.eq(0),
        ]

        command = Signal(8)

        m.d.comb += self.leds.eq(Cat(
            rx_stretch != 0,   # 0: a command byte arrived recently
            self.tx_active,    # 1: transmitting a response
            last_ok,           # 2: last command understood
            last_power,        # 3: last command was POWER
            heartbeat,         # 4: toggles per response
            unknown_cmd,       # 5: last command rejected
        ))

        with m.FSM():

            with m.State("IDLE"):
                m.d.comb += self.tx_active.eq(0)
                with m.If(rx_uart.strobe):
                    m.d.sync += command.eq(rx_uart.data)
                    m.d.comb += crc.clear.eq(1)

                    with m.Switch(rx_uart.data):
                        with m.Case(CMD_PING):
                            m.d.sync += [ok.eq(1),
                                         payload_len.eq(PAYLOAD_SIZE[CMD_PING]),
                                         last_power.eq(0), unknown_cmd.eq(0)]
                        with m.Case(CMD_STATUS):
                            m.d.sync += [ok.eq(1),
                                         payload_len.eq(PAYLOAD_SIZE[CMD_STATUS]),
                                         last_power.eq(0), unknown_cmd.eq(0)]
                        with m.Case(CMD_POWER):
                            m.d.sync += [ok.eq(1),
                                         payload_len.eq(PAYLOAD_SIZE[CMD_POWER]),
                                         last_power.eq(1), unknown_cmd.eq(0)]
                        with m.Default():
                            # Unknown command: status alone, OK clear. The
                            # master still gets a well-formed reply rather than
                            # a timeout, so it can tell "not understood" from
                            # "not there".
                            m.d.sync += [ok.eq(0), payload_len.eq(0),
                                         unknown_cmd.eq(1)]

                    m.d.sync += [byte_index.eq(0), turnaround.eq(self.divisor * 2)]
                    m.next = "TURNAROUND"

            # Wait two bit periods before replying.
            #
            # The receiver frames a byte at the MIDDLE of its stop bit, but the
            # master needs until the END of that bit -- plus its own handover
            # time -- before it can listen. Apollo bit-bangs its transmit and
            # only returns the pin to its UART one bit period after the stop
            # bit, so replying immediately loses the first bits of the response.
            # Observed as the FPGA receiving every command correctly while the
            # master saw nothing.
            with m.State("TURNAROUND"):
                m.d.comb += self.tx_active.eq(0)
                m.d.sync += turnaround.eq(turnaround - 1)
                with m.If(turnaround == 0):
                    m.next = "SETTLE"

            # One cycle for the sync writes above (ok, payload_len, command) to
            # land. status is combinational, so sampling it in the very next
            # cycle would capture the previous command's flags -- which showed
            # up as the OK bit reading zero on a valid command.
            with m.State("SETTLE"):
                m.d.comb += self.tx_active.eq(1)
                m.next = "SEND_STATUS"

            with m.State("SEND_STATUS"):
                m.d.comb += self.tx_active.eq(1)
                with m.If(~tx_uart.busy):
                    m.d.sync += send_byte.eq(status)
                    m.d.comb += [
                        crc.data.eq(status),
                        crc.strobe.eq(1),
                        tx_uart.start.eq(1),
                    ]
                    m.next = "SEND_STATUS_WAIT"

            with m.State("SEND_STATUS_WAIT"):
                m.d.comb += self.tx_active.eq(1)
                with m.If(tx_uart.busy):
                    m.next = "PAYLOAD"

            with m.State("PAYLOAD"):
                m.d.comb += self.tx_active.eq(1)
                with m.If(byte_index == payload_len):
                    m.next = "SEND_CRC"
                with m.Elif(~tx_uart.busy):
                    selected = Signal(8)
                    with m.If(command == CMD_POWER):
                        m.d.comb += selected.eq(
                            self.power_data.word_select(byte_index[:4], 8))
                    with m.Else():
                        m.d.comb += selected.eq(ping_payload[byte_index[:1]])

                    m.d.sync += send_byte.eq(selected)
                    m.d.comb += [
                        crc.data.eq(selected),
                        crc.strobe.eq(1),
                        tx_uart.start.eq(1),
                    ]
                    m.d.sync += byte_index.eq(byte_index + 1)
                    m.next = "PAYLOAD_WAIT"

            with m.State("PAYLOAD_WAIT"):
                m.d.comb += self.tx_active.eq(1)
                with m.If(tx_uart.busy):
                    m.next = "PAYLOAD"

            with m.State("SEND_CRC"):
                m.d.comb += self.tx_active.eq(1)
                with m.If(~tx_uart.busy):
                    m.d.sync += send_byte.eq(crc.crc)
                    m.d.comb += tx_uart.start.eq(1)
                    m.next = "DRAIN"

            with m.State("DRAIN"):
                m.d.comb += self.tx_active.eq(1)
                # Hold tx_active until the final stop bit has actually gone out,
                # or releasing the line would truncate it.
                with m.If(tx_uart.busy):
                    m.next = "FINISH"

            with m.State("FINISH"):
                m.d.comb += self.tx_active.eq(1)
                with m.If(~tx_uart.busy):
                    m.d.sync += [heartbeat.eq(~heartbeat), last_ok.eq(ok)]
                    m.next = "IDLE"

        return m
