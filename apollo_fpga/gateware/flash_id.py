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

from amaranth import Elaboratable, Module, Mux, Signal


# Winbond/JEDEC opcodes. 0x9F is the one command every SPI-NOR part answers the
# same way, which is why identification uses it rather than anything vendor
# specific.
OPCODE_JEDEC_ID  = 0x9F
OPCODE_READ      = 0x03   # up to 50 MHz on the W25Q32DV
OPCODE_FAST_READ = 0x0B   # up to 104 MHz, one dummy byte after the address


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

    def __init__(self, *, spi_controller):
        self.spi = spi_controller

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
        XOR of every byte read, so a transfer that returns garbage at speed is
        distinguishable from one that returns real data.
    done : Signal(), out
        High once a measurement has completed.
    """

    def __init__(self, *, spi_controller):
        self.spi = spi_controller

        self.start     = Signal()
        self.length    = Signal(16)
        self.fast_read = Signal()
        self.cycles    = Signal(32)
        self.checksum  = Signal(8)
        self.done      = Signal()
        self.busy      = Signal()

    def elaborate(self, platform):
        m = Module()

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
                        self.checksum.eq(0),
                        self.done   .eq(0),
                    ]
                    m.next = "HEADER"

            with m.State("HEADER"):
                m.d.comb += [
                    self.busy             .eq(1),
                    self.spi.input.valid  .eq(1),
                    self.spi.input.payload.eq(header_byte),
                    self.spi.input.last   .eq(0),
                ]
                m.d.sync += self.cycles.eq(self.cycles + 1)
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
                        m.d.sync += self.checksum.eq(
                            self.checksum ^ self.spi.output.payload)
                    m.d.sync += received.eq(received + 1)
                    with m.If(self.spi.output.last):
                        m.d.sync += self.done.eq(1)
                        m.next = "IDLE"

        return m
