#
# This file is part of Apollo.
#
# SPDX-License-Identifier: BSD-3-Clause

""" FPGA_ADV command responder.

Half-duplex request/response over the single FPGA_ADV wire, with Apollo as the
master. See docs/apollo_samd11_mcu/fpga-adv-sideband.md.

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
CMD_DEVICES = 0x2C   # flash JEDEC ID + HyperRAM presence

# LED control occupies 0x40-0x7F: the low six bits are the pattern, so the
# whole command fits in the single opcode byte and the protocol stays
# stateless. 0x40 turns them all off, 0x7F turns them all on.
CMD_LED_BASE = 0x40
CMD_LED_MASK = 0x3F

# Releases the LEDs back to whatever the design drives by default.
#
# Without this the override is irreversible: it latches on any opcode in the
# 0x40-0x7F range and nothing ever clears it. That range is a quarter of all
# byte values, so a single framing error whose garbage lands in it hijacks the
# display until the FPGA is reconfigured. Framing errors are now rejected
# upstream, which removes the accidental path -- but an override with no
# release is still a state the link cannot get out of, and the protocol
# document described the command as latching without saying it was permanent.
CMD_LED_RELEASE = 0x03

# Response payload sizes, excluding the status byte and the CRC.
PAYLOAD_SIZE = {
    CMD_PING:   2,
    CMD_STATUS: 0,
    CMD_POWER:  16,
    CMD_DEVICES: 4,   # flash manufacturer, type, capacity, then a flags byte
}

# Status byte bits.
STATUS_OK          = 0
STATUS_EVENTS      = 1
STATUS_ERROR       = 2
STATUS_RECONFIG    = 3
STATUS_STATE_SHIFT = 4
STATUS_HEARTBEAT   = 6

PROTOCOL_VERSION = 0x01

# The firmware's response buffer size, from ADV_RESPONSE_MAX in
# firmware/src/boards/cynthion_d11/fpga_adv.c. A reply is status + payload +
# CRC, so the largest payload this buffer can carry is ADV_RESPONSE_MAX - 2.
#
# Duplicated rather than shared because one side is C on a Cortex-M0+ and the
# other is Python, and there is no build step that sees both. The assertion in
# SidebandResponder.elaborate() is what makes the duplication safe: adding a
# 17-byte command here would otherwise truncate silently in gateware while
# firmware happily accepted the longer reply, and nothing tied the two numbers
# together.
FIRMWARE_ADV_RESPONSE_MAX = 18
MAX_PAYLOAD = FIRMWARE_ADV_RESPONSE_MAX - 2


class CRC8(Elaboratable):
    """ CRC-8/ATM: polynomial 0x07, init 0x00, no reflection, no final XOR.

    One byte at a time, eight shifts per clock. At the link's 230400 baud a byte
    takes ~43 us, so an eight-deep combinational chain is never the critical
    path.
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
        # Frames rejected for a bad stop bit. Exposed rather than merely
        # discarded, because a count that climbs is the difference between "the
        # link is quiet" and "the link is receiving noise" -- and those want
        # very different responses.
        self.errors  = Signal(8)

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
            #
            # Gated on err.frame. AsyncSerialRX strobes rdy for *every*
            # completed frame, including one whose stop bit was low, and
            # err.frame is the only thing distinguishing a byte from noise.
            # Without this gate a break condition -- the line held low, which
            # is exactly what happens while Apollo re-muxes the pin or the FPGA
            # reconfigures -- frames as several garbage bytes, each dispatched
            # as a command and each answered with an unsolicited transmission
            # onto a wire the master may be driving.
            self.strobe.eq(rx.rdy & rx.ack & ~rx.err.frame),
        ]

        # Saturating, so a counter pinned at 255 still reads as "bad" where a
        # wrapped one could read as healthy.
        with m.If(rx.rdy & rx.err.frame & (self.errors != self.errors.all())):
            m.d.sync += self.errors.eq(self.errors + 1)

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
        O: pad_o        -- wire straight to pad.o; respects open_drain
        O: pad_oe       -- wire straight to pad.oe; respects open_drain
        I: power_data   -- 128 bits: VBUS[0..3] then VSENSE[0..3], LE16 each
        I: events       -- events pending
        I: error        -- error flag
        I: reconfigured -- FPGA reconfigured since last poll
        I: state        -- 2-bit FPGA state
    """

    # baud defaults to 230400, matching ADV_UART_BAUD in the Apollo firmware.
    # A mismatch here kills the link silently -- both ends simply never frame a
    # byte, with no error raised anywhere -- so the default must be the shipping
    # rate rather than the historical one.
    def __init__(self, clk_freq_hz=60e6, baud=230400, turnaround_us=40,
                 open_drain=True):
        # The bit period is a cycle count fixed at build time, so this module is
        # only correct if the domain it ends up in really runs at clk_freq_hz.
        # Nothing checks that at elaboration: a design that raises its sync
        # frequency and leaves this argument alone gets a link that is dead
        # rather than degraded -- at 120 MHz the effective baud doubles, and a
        # UART tolerates roughly +/-2%.
        #
        # Instantiate under DomainRenamer onto a domain pinned to this
        # frequency when the rest of the design clocks faster; see the sideband
        # test bitstream for the pattern.
        self.clk_freq_hz = clk_freq_hz
        self.baud = baud
        self.divisor = int(clk_freq_hz // baud)

        if self.divisor < 8:
            raise ValueError(
                f"clock {clk_freq_hz/1e6:.1f} MHz is too slow for {baud} baud: "
                f"divisor {self.divisor} leaves no room to sample mid-bit")

        # Turnaround is an ABSOLUTE time, not a multiple of the bit period.
        #
        # Measured on r1.4 at 115200: 20 us fails (the master's stop bit is
        # still on the wire when the receiver re-arms, so the tail of the
        # previous frame is taken as a start bit and the next byte reads one
        # bit position late -- 0x02 framed as 0x80). 25 us and above pass
        # 20/20. The default is 40 us, twice the observed failure point, so the
        # margin does not depend on a boundary measured on one board.
        #
        # The master's handover cost is fixed in CPU cycles -- Apollo bit-bangs
        # its transmit from a timer ISR and only returns the pin to its SERCOM
        # one bit period after the stop bit, then the SERCOM must resync. None
        # of that shrinks as the baud rises. Scaling the delay with the bit
        # period therefore starved it exactly when it was needed most: 100% at
        # 115200, 92% at 230400, 0% above.
        self.turnaround_cycles = max(int(clk_freq_hz * turnaround_us / 1e6), 1)

        # Open-drain by default: the FPGA only ever pulls FPGA_ADV low and lets
        # the pull-ups raise it. See pad_o/pad_oe below for why that is the
        # default rather than an option.
        self.open_drain = open_drain

        self.rx           = Signal(init=1)
        self.tx           = Signal(init=1)
        self.tx_active    = Signal()

        # Drive FPGA_ADV through these, not through tx/tx_active.
        #
        # FPGA_ADV is one wire with a CMOS driver at each end. Driving it
        # push-pull means a timing slip that enables both drivers at once puts a
        # driven high against a driven low: a low-impedance path through two
        # output stages, tens to hundreds of milliamps, and a hardware-damage
        # risk rather than a link-reliability one. Open-drain removes the
        # possibility rather than narrowing the window -- two drivers pulling
        # low together is just a low.
        #
        # The gap this closes is not hypothetical. tx_active asserts one full
        # SETTLE-plus-LOAD ahead of the first bit reaching the wire, so
        # oe = tx_active with o = tx drives the line HIGH for that lead-in,
        # while the master's own driver may still be enabled. Measured at 0.75
        # bit times by scripts/sideband_contention_probe.py.
        #
        # Exposed as pad signals rather than left to each consumer: every
        # consumer that wired oe = tx_active got push-pull by accident, and the
        # decision belongs in one place.
        self.pad_o        = Signal()
        self.pad_oe       = Signal()

        self.power_data   = Signal(128)

        # Momentary inputs, latched. A button press between two polls would be
        # missed entirely if the master only ever saw the live level, so an
        # edge sets a sticky bit that survives until the master has actually
        # seen it.
        self.button       = Signal()

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

        # Host-commanded LED pattern. led_override latches on the first LED
        # command so the display does not flicker back to its default state
        # between commands, and is cleared by CMD_LED_RELEASE.
        self.led_pattern  = Signal(6)
        self.led_override = Signal()

        # Device identification, driven by FlashIDReader and the HyperRAM
        # block. Sampled when a DEVICES command arrives rather than being
        # buffered here, since none of it changes while powered.
        self.flash_manufacturer = Signal(8)
        self.flash_memory_type  = Signal(8)
        self.flash_capacity     = Signal(8)
        self.flash_valid        = Signal()
        self.hyperram_present   = Signal()

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

        # Pad driving, and the whole of issue #88 in four lines.
        #
        # Open-drain: never drive high, only pull low, and only while we own the
        # line. oe therefore tracks the bit value, not merely tx_active, so the
        # driver is off for every high bit and for the entire lead-in before the
        # start bit. Contention becomes impossible by construction: whatever the
        # master does, the worst case is two drivers pulling the same wire low.
        #
        # Push-pull is kept reachable, because open-drain trades drive strength
        # for rise time and that trade has NOT been measured on this board.
        #
        # The falling edge is unchanged -- an active pull-down either way. The
        # rising edge stops being a driven transition and becomes an RC against
        # the two internal pull-ups in parallel. Order of magnitude, and this is
        # arithmetic rather than measurement:
        #
        #   Apollo's PA09 pull is specified 20/40/60 kOhm. The ECP5's is a weak
        #   current source of tens of microamps, not a stated resistance, so the
        #   parallel figure is unknown but bounded below by the SAMD11's worst
        #   case: >= 60 kOhm is pessimistic, ~20 kOhm optimistic. Load is the two
        #   pin capacitances plus the trace, call it 15-25 pF. One RC is then
        #   0.3-1.5 us, and reaching a CMOS threshold takes roughly one RC.
        #
        #   At 230400 baud a bit is 4.34 us, so 0.3-1.5 us of rise is 7-35% of a
        #   bit. The receiver samples at bit centre, so the low end is
        #   comfortable and the high end is marginal. At 115200 (8.68 us bits)
        #   the same rise is 3-17% and comfortable throughout.
        #
        # That spread is why this is a real risk and not a formality: the
        # pessimistic end of an unmeasured range lands close enough to matter at
        # 230400. It cannot be settled in simulation -- it needs a scope on T6,
        # or the open-drain build of ecp5-test/adv_speed, which exists precisely
        # to find the crossover and whose result was never applied.
        #
        # Correctness wins over rate here regardless: push-pull at both ends is a
        # hardware-damage path, and dropping to 115200 is a recoverable cost
        # where a shorted driver is not. If the link proves marginal at 230400
        # once open-drain, drop BOTH sides to 115200 together -- see the baud
        # note on __init__.
        if self.open_drain:
            m.d.comb += [
                self.pad_o.eq(0),
                self.pad_oe.eq(self.tx_active & ~self.tx),
            ]
        else:
            m.d.comb += [
                self.pad_o.eq(self.tx),
                self.pad_oe.eq(self.tx_active),
            ]

        # Flips on every response: a value that *changes* proves the responder
        # is executing, where a repeated value could be a wedged state machine
        # replaying a stale buffer.
        heartbeat = Signal()

        #
        # Button latch, read-and-clear.
        #
        # Set on a rising edge of self.button, reported in the status byte, and
        # cleared only once a response carrying it has been fully transmitted.
        # Clearing on receipt of the command instead would lose the press if
        # the reply were then lost -- the master would never learn of it.
        #
        # A press arriving mid-response sets the latch again rather than being
        # swallowed, since the set takes priority over the clear below.
        #
        button_latch = Signal()
        button_prev  = Signal()
        button_seen  = Signal()   # this response is reporting the latch

        m.d.sync += button_prev.eq(self.button)
        with m.If(self.button & ~button_prev):
            m.d.sync += button_latch.eq(1)

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
            self.events | button_latch,   # bit 1: events, incl. button press
            self.error,         # bit 2
            self.reconfigured,  # bit 3
            self.state,         # bits 4-5
            heartbeat,          # bit 6
            Const(0, 1),        # bit 7
        ))

        # Payload bytes for the command in flight. Sized for the largest
        # response so one register file serves all commands.
        #
        # Widths derive from MAX_PAYLOAD rather than being written out, and the
        # assertion is what ties this side to the firmware's buffer. Previously
        # payload_len was range(17) against a largest payload of 16 -- correct,
        # but with zero headroom and nothing checking it, so a 17-byte command
        # would have truncated here while firmware accepted the longer reply.
        # This is an elaboration-time failure rather than a wrong bitstream.
        largest = max(PAYLOAD_SIZE.values())
        assert largest <= MAX_PAYLOAD, (
            f"largest payload {largest} exceeds what the firmware's "
            f"{FIRMWARE_ADV_RESPONSE_MAX}-byte response buffer can carry "
            f"({MAX_PAYLOAD}); raise ADV_RESPONSE_MAX in "
            f"firmware/src/boards/cynthion_d11/fpga_adv.c and "
            f"FIRMWARE_ADV_RESPONSE_MAX here together")

        turnaround  = Signal(range(self.turnaround_cycles + 1))
        # +1 so payload_len can hold MAX_PAYLOAD itself.
        payload_len = Signal(range(MAX_PAYLOAD + 1))
        # Counts status + payload + CRC, so it must reach the full reply length.
        byte_index  = Signal(range(FIRMWARE_ADV_RESPONSE_MAX + 1))
        send_byte   = Signal(8)

        # PING's payload; POWER's comes from power_data.
        ping_payload = Array([Const(PROTOCOL_VERSION, 8), Const(0x00, 8)])

        # Flash JEDEC ID, then a flags byte. The flags carry presence rather
        # than identity: HyperRAM has no JEDEC ID to read, so the only useful
        # thing to report is whether it answered at all.
        device_flags = Signal(8)
        m.d.comb += device_flags.eq(Cat(self.hyperram_present,
                                        self.flash_valid,
                                        Const(0, 6)))
        devices_payload = Array([self.flash_manufacturer,
                                 self.flash_memory_type,
                                 self.flash_capacity,
                                 device_flags])

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
                        with m.Case(CMD_LED_RELEASE):
                            m.d.sync += [ok.eq(1), payload_len.eq(0),
                                         last_power.eq(0), unknown_cmd.eq(0),
                                         self.led_override.eq(0),
                                         self.led_pattern.eq(0)]
                        with m.Case(CMD_DEVICES):
                            # OK reflects whether the flash ID has actually been
                            # read, so a host cannot mistake the power-on zeros
                            # for a device that answered with zeros.
                            m.d.sync += [ok.eq(self.flash_valid),
                                         payload_len.eq(PAYLOAD_SIZE[CMD_DEVICES]),
                                         last_power.eq(0), unknown_cmd.eq(0)]
                        with m.Default():
                            # LED control: 0x40-0x7F, pattern in the low six
                            # bits. Ranged rather than enumerated so all 64
                            # patterns are one case.
                            with m.If((rx_uart.data & ~CMD_LED_MASK)
                                      == CMD_LED_BASE):
                                m.d.sync += [
                                    ok.eq(1), payload_len.eq(0),
                                    last_power.eq(0), unknown_cmd.eq(0),
                                    self.led_override.eq(1),
                                    self.led_pattern.eq(
                                        rx_uart.data & CMD_LED_MASK),
                                ]
                            with m.Else():
                                # Unknown command: status alone, OK clear. The
                                # master still gets a well-formed reply rather
                                # than a timeout, so it can tell "not
                                # understood" from "not there".
                                m.d.sync += [ok.eq(0), payload_len.eq(0),
                                             unknown_cmd.eq(1)]

                    m.d.sync += [byte_index.eq(0),
                                 turnaround.eq(self.turnaround_cycles),
                                 # Remember whether this reply will carry the
                                 # latch, so it is cleared only if it did.
                                 button_seen.eq(button_latch)]
                    m.next = "TURNAROUND"

            # Wait a fixed turnaround time before replying.
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
                    with m.Elif(command == CMD_DEVICES):
                        m.d.comb += selected.eq(
                            devices_payload[byte_index[:2]])
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
                    # Clear only what this response actually reported. A press
                    # that arrived while transmitting keeps its latch set,
                    # because the set above wins over this clear.
                    with m.If(button_seen):
                        m.d.sync += [button_latch.eq(0), button_seen.eq(0)]
                    m.next = "IDLE"

        return m
