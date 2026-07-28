#
# This file is part of Apollo.
#
# Copyright (c) 2020-2026 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" Configuration-flash identification and read-throughput measurement.

Both of these exist to be served over the FPGA_ADV sideband link, so that a
unit running full USB emulation can still be asked what flash it has and how
fast that flash is actually reading -- neither of which the host-facing USB can
answer while the FPGA is busy being an emulated device.

The SPI master is Apollo's own ``SPIStreamController`` and the pin-level
adaptation is LUNA's ``ECP5ConfigurationFlashInterface``; neither is
reimplemented here. The ECP5 cannot drive its configuration clock from ordinary
fabric -- SCK has to come out through the ``USRMCLK`` primitive -- and that is
precisely the detail the LUNA block exists to hide.
"""

from amaranth import Elaboratable, Module, Mux, Signal, unsigned
from amaranth.lib.memory import Memory

from luna.gateware.stream import StreamInterface

from .sideband import CRC8


# Winbond/JEDEC opcodes. 0x9F is the one command every SPI-NOR part answers the
# same way, which is why identification uses it rather than anything vendor
# specific.
OPCODE_JEDEC_ID  = 0x9F
OPCODE_READ      = 0x03   # up to 50 MHz on the W25Q32DV
OPCODE_FAST_READ = 0x0B   # up to 104 MHz, one dummy byte after the address


class SPIPort:
    """ One requester's private view of the SPI controller's streams.

    Each block drives its own port and a mux grants the real controller to
    whichever is active. Without this both blocks drive the controller's input
    stream directly and Amaranth rejects the design with a DriverConflict --
    correctly, since combinational drivers would otherwise be merged into
    nonsense.
    """

    def __init__(self):
        self.input  = StreamInterface()
        self.output = StreamInterface()


class SPIMux(Elaboratable):
    """ Grants a shared SPIStreamController to one of several ports.

    Ownership here is strictly sequential -- the ID read completes before the
    throughput measurement begins -- so this is a selector rather than an
    arbiter: it does not queue, and a request made while another port holds the
    bus is simply not connected. Two blocks that genuinely overlapped would
    need real arbitration.
    """

    def __init__(self, *, controller, ports):
        self.controller = controller
        self.ports      = list(ports)
        self.select     = Signal(range(max(len(self.ports), 2)))

    def elaborate(self, platform):
        m = Module()

        ctrl = self.controller
        with m.Switch(self.select):
            for index, port in enumerate(self.ports):
                with m.Case(index):
                    m.d.comb += [
                        ctrl.input.valid   .eq(port.input.valid),
                        ctrl.input.payload .eq(port.input.payload),
                        ctrl.input.last    .eq(port.input.last),
                        port.input.ready   .eq(ctrl.input.ready),

                        port.output.valid  .eq(ctrl.output.valid),
                        port.output.payload.eq(ctrl.output.payload),
                        port.output.last   .eq(ctrl.output.last),
                        ctrl.output.ready  .eq(port.output.ready),
                    ]

        return m


class FlashIDReader(Elaboratable):
    """ Reads the three-byte JEDEC ID from the configuration flash.

    Runs a single transaction on request and then holds the result. The ID
    cannot change while the board is powered, so the sideband can read the
    latched value without ever waiting on SPI.

    Attributes
    ----------
    start : Signal(), in
        Strobe to begin a read. Ignored while busy.
    manufacturer : Signal(8), out
        JEDEC manufacturer ID; 0xEF for Winbond.
    memory_type : Signal(8), out
        Device type byte.
    capacity : Signal(8), out
        Capacity byte; log2 of the size in bytes, so 0x16 is 4 MiB.
    valid : Signal(), out
        High once a read has completed.
    """

    def __init__(self):
        self.spi = SPIPort()

        self.start        = Signal()
        self.manufacturer = Signal(8)
        self.memory_type  = Signal(8)
        self.capacity     = Signal(8)
        self.valid        = Signal()
        self.busy         = Signal()

    def elaborate(self, platform):
        m = Module()

        # Four bytes on the wire: the opcode, then three dummy bytes clocked out
        # purely to generate the clocks the device needs to return its ID. SPI
        # cannot read without writing.
        byte_index = Signal(range(4))
        collecting = Signal()

        # Capture runs in parallel with transmission, not after it. The
        # controller's output.valid is a one-cycle registered strobe that fires
        # as each byte finishes shifting in, so the first three returns arrive
        # while bytes are still being handed to the input stream. An FSM that
        # waits until the last byte has been accepted before it starts looking
        # will have missed them -- which is what this cost the first time.
        with m.If(collecting & self.spi.output.valid):
            m.d.sync += [
                self.manufacturer.eq(self.memory_type),
                self.memory_type .eq(self.capacity),
                self.capacity    .eq(self.spi.output.payload),
            ]
            # The response to the opcode is meaningless, but shifting all four
            # captures through drops it off the far end without a counter.
            with m.If(self.spi.output.last):
                m.d.sync += [
                    self.valid.eq(1),
                    collecting.eq(0),
                ]

        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.start):
                    m.d.sync += [
                        byte_index.eq(0),
                        collecting.eq(1),
                    ]
                    m.next = "SEND"

            with m.State("SEND"):
                m.d.comb += [
                    self.busy             .eq(1),
                    self.spi.input.valid  .eq(1),
                    # The opcode first, then zeros. The payload of the dummy
                    # bytes is irrelevant; only the clocking matters.
                    self.spi.input.payload.eq(
                        Mux(byte_index == 0, OPCODE_JEDEC_ID, 0)),
                    # Deassert CS after the fourth byte, ending the transaction.
                    self.spi.input.last   .eq(byte_index == 3),
                ]
                with m.If(self.spi.input.ready):
                    with m.If(byte_index == 3):
                        m.next = "DRAIN"
                    with m.Else():
                        m.d.sync += byte_index.eq(byte_index + 1)

            with m.State("DRAIN"):
                # The last byte is still shifting in; the capture block above
                # finishes the job and clears `collecting`.
                m.d.comb += self.busy.eq(collecting)
                with m.If(~collecting):
                    m.next = "IDLE"

        return m


class FlashStatusReader(Elaboratable):
    """ Reads the three status registers.

    Register 2 carries the Quad Enable bit, which must be set before any quad
    read will work -- and setting it is not free: with QE=1 the /WP and /HOLD
    pins become IO2 and IO3, so hardware write protection is gone. Reading the
    bit before deciding anything is therefore worth a block of its own.

    Attributes
    ----------
    start : Signal(), in
        Strobe to read all three registers.
    status : Signal(24), out
        Registers 1, 2 and 3, least-significant byte first.
    valid : Signal(), out
        High once a read has completed.
    """

    # Read Status Register-1 (05h), -2 (35h) and -3 (15h).
    OPCODES = (0x05, 0x35, 0x15)

    def __init__(self):
        self.spi = SPIPort()

        self.start  = Signal()
        self.status = Signal(24)
        self.valid  = Signal()
        self.busy   = Signal()

    def elaborate(self, platform):
        m = Module()

        which    = Signal(range(len(self.OPCODES)))
        opcode   = Signal(8)
        got_byte = Signal()

        with m.Switch(which):
            for index, code in enumerate(self.OPCODES):
                with m.Case(index):
                    m.d.comb += opcode.eq(code)

        # Each register is a two-byte transaction: opcode out, value back. The
        # value arrives on the second byte, so the first return is discarded.
        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.start):
                    m.d.sync += [which.eq(0), got_byte.eq(0)]
                    m.next = "OPCODE"

            with m.State("OPCODE"):
                m.d.comb += [
                    self.busy             .eq(1),
                    self.spi.input.valid  .eq(1),
                    self.spi.input.payload.eq(opcode),
                    self.spi.input.last   .eq(0),
                ]
                with m.If(self.spi.input.ready):
                    m.next = "VALUE"

            with m.State("VALUE"):
                m.d.comb += [
                    self.busy             .eq(1),
                    self.spi.input.valid  .eq(~got_byte),
                    self.spi.input.payload.eq(0),
                    self.spi.input.last   .eq(1),
                ]
                with m.If(self.spi.input.valid & self.spi.input.ready):
                    m.d.sync += got_byte.eq(1)

                with m.If(self.spi.output.valid & self.spi.output.last):
                    m.d.sync += [
                        self.status.word_select(which, 8)
                            .eq(self.spi.output.payload),
                        got_byte.eq(0),
                    ]
                    with m.If(which == len(self.OPCODES) - 1):
                        m.d.sync += self.valid.eq(1)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += which.eq(which + 1)
                        m.next = "OPCODE"

        return m


class FlashCapture(Elaboratable):
    """ Buffers the first N bytes of a flash read into block RAM.

    A checksum says only that something is wrong. This says *what* is wrong:
    the bytes come back verbatim, so a read that fails at speed can be compared
    against the same read at a known-good rate, byte for byte. That distinguishes
    a shifted sample point (data present but rotated), a dead MISO (all zeros or
    all ones) and genuine corruption -- which a single fold cannot.

    Sized in bytes; the ECP5-12F has 56 DP16KD blocks, so a few KiB is cheap.

    Attributes
    ----------
    write_strobe : Signal(), in
        Assert with `write_data` valid to append a byte.
    write_data : Signal(8), in
        The byte to append.
    clear : Signal(), in
        Reset the write pointer to zero.
    read_addr : Signal(), in
        Byte address to read back.
    read_data : Signal(8), out
        Contents of `read_addr`, one cycle later.
    """

    def __init__(self, *, depth=1024):
        self.depth = depth

        self.write_strobe = Signal()
        self.write_data   = Signal(8)
        self.clear        = Signal()
        self.read_addr    = Signal(range(depth))
        self.read_data    = Signal(8)
        self.count        = Signal(range(depth + 1))

    def elaborate(self, platform):
        m = Module()

        memory = Memory(shape=unsigned(8), depth=self.depth, init=[])
        m.submodules.memory = memory

        write_port = memory.write_port()
        read_port  = memory.read_port(domain="sync")

        m.d.comb += [
            write_port.addr.eq(self.count),
            write_port.data.eq(self.write_data),
            # Stop at the end rather than wrapping: a wrapped buffer silently
            # shows the tail of a long read where the head is what was asked for.
            write_port.en  .eq(self.write_strobe & (self.count < self.depth)),

            read_port.addr .eq(self.read_addr),
            self.read_data .eq(read_port.data),
        ]

        with m.If(self.clear):
            m.d.sync += self.count.eq(0)
        with m.Elif(self.write_strobe & (self.count < self.depth)):
            m.d.sync += self.count.eq(self.count + 1)

        return m


class FlashSpeedTest(Elaboratable):
    """ Measures sustained sequential read throughput from the flash.

    Reads a fixed span and counts sync-domain cycles between the first clock and
    the last returned byte, so the figure includes the opcode, the address and
    the per-transaction overhead rather than only the payload -- that is what a
    caller actually experiences.

    The result is reported in cycles rather than converted to bytes per second
    here: the divisor and the sync clock are both known to the host, and doing
    the arithmetic in gateware would cost a divider to produce a number that is
    less precise than the raw count.

    Attributes
    ----------
    start : Signal(), in
        Strobe to begin a measurement.
    length : Signal(16), in
        Number of bytes to read.
    fast_read : Signal(), in
        Use 0x0B (with its dummy byte) instead of 0x03.
    cycles : Signal(32), out
        Sync-domain cycles the transfer occupied.
    checksum : Signal(8), out
        Position-weighted checksum of every byte read, so a transfer that
        returns garbage at speed is distinguishable from one that returns real
        data.

        A plain XOR is not adequate here: it cancels itself on any even-length
        run of constant data, so a 4096-byte read of all 0x00 and a 4096-byte
        read of all 0xFF both fold to 0x00 -- and so does a read that returned
        nothing at all. Rotating the accumulator before each fold makes the
        result depend on byte order and length, so those cases separate.
    done : Signal(), out
        High once a measurement has completed.
    """

    def __init__(self):
        self.spi = SPIPort()

        self.start     = Signal()
        self.length    = Signal(16)
        self.fast_read = Signal()
        self.cycles    = Signal(32)
        self.checksum  = Signal(8)
        self.done      = Signal()
        self.busy      = Signal()

        # Payload bytes as they arrive, for an optional capture buffer. Taps
        # the same point the checksum folds, so what is captured is exactly
        # what is summed.
        self.data_strobe = Signal()
        self.data        = Signal(8)

    def elaborate(self, platform):
        m = Module()

        # CRC-8 rather than a checksum. An XOR fold cancels on any even-length
        # run of constant data, and a rotate-and-fold cancels the same way
        # whenever the run length is a multiple of the rotation period -- so
        # 4096 bytes of 0x00, of 0xFF, and a transfer that returned nothing all
        # produced the same 0x00 and the detector could not tell them apart.
        # The polynomial makes the result depend on both content and length.
        m.submodules.crc = crc = CRC8()
        m.d.comb += self.checksum.eq(crc.crc)

        # Header is opcode + three address bytes, plus one dummy byte for the
        # fast-read variant, which is the whole reason fast read can clock
        # faster: the extra byte gives the die time to reach the first address.
        header_len  = Signal(range(6))
        header_sent = Signal(range(6))
        bytes_left  = Signal(16)
        received    = Signal(16)

        m.d.comb += header_len.eq(Mux(self.fast_read, 5, 4))

        # The opcode, then a zero address, then the dummy byte if any: reading
        # from address zero keeps the header a simple index lookup.
        header_byte = Mux(header_sent == 0,
                          Mux(self.fast_read, OPCODE_FAST_READ, OPCODE_READ),
                          0)

        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.start):
                    m.d.sync += [
                        header_sent .eq(0),
                        bytes_left  .eq(self.length),
                        received    .eq(0),
                        self.cycles .eq(0),
                        self.done   .eq(0),
                    ]
                    m.d.comb += crc.clear.eq(1)
                    m.next = "HEADER"

            with m.State("HEADER"):
                m.d.comb += [
                    self.busy             .eq(1),
                    self.spi.input.valid  .eq(1),
                    self.spi.input.payload.eq(header_byte),
                    self.spi.input.last   .eq(0),
                ]
                m.d.sync += self.cycles.eq(self.cycles + 1)

                # The device returns a byte for every byte sent, so the header
                # generates returns too -- and they arrive here, in HEADER,
                # while the header is still going out. Counting them only in
                # PAYLOAD leaves `received` short by the header length, so the
                # gate below discards real payload bytes instead of the
                # header's. On an even-length read that silently folds the
                # checksum to zero, which reads exactly like a corrupt
                # transfer: the bug this cost was diagnosed as a hardware
                # failure at 60 MHz before a constant-data simulation showed
                # the count, not the data, was wrong.
                with m.If(self.spi.output.valid):
                    m.d.sync += received.eq(received + 1)

                with m.If(self.spi.input.ready):
                    with m.If(header_sent == header_len - 1):
                        m.next = "PAYLOAD"
                    with m.Else():
                        m.d.sync += header_sent.eq(header_sent + 1)

            with m.State("PAYLOAD"):
                m.d.comb += [
                    self.busy             .eq(1),
                    self.spi.input.valid  .eq(bytes_left != 0),
                    self.spi.input.payload.eq(0),
                    self.spi.input.last   .eq(bytes_left == 1),
                ]
                m.d.sync += self.cycles.eq(self.cycles + 1)

                with m.If(self.spi.input.valid & self.spi.input.ready):
                    m.d.sync += bytes_left.eq(bytes_left - 1)

                # Count and fold in only the payload bytes; the header responses
                # are whatever the device happened to be driving.
                with m.If(self.spi.output.valid):
                    with m.If(received >= header_len):
                        m.d.comb += [
                            crc.data.eq(self.spi.output.payload),
                            crc.strobe.eq(1),
                            self.data_strobe.eq(1),
                            self.data.eq(self.spi.output.payload),
                        ]
                    m.d.sync += received.eq(received + 1)
                    with m.If(self.spi.output.last):
                        m.d.sync += self.done.eq(1)
                        m.next = "IDLE"

        return m
