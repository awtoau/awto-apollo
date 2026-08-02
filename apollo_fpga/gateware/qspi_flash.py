#
# This file is part of Apollo.
#
# Copyright (c) 2020-2026 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" Quad-SPI access to the ECP5 configuration flash.

The QSPI controller itself is Glasgow's (``glasgow.gateware.qspi``, 0BSD): it
handles x1/x2/x4 transfers in both directions, targets Lattice parts natively,
and exposes a configurable sample offset -- the knob our single-lane
implementation lacked when it failed at 60 MHz SCK. There is no quad SPI core
in Amaranth itself, in amaranth-soc, or in LUNA, so the alternative was writing
one from scratch.

What Glasgow cannot know about is the ECP5's configuration flash clock. SCK has
no ordinary I/O buffer -- it belongs to the configuration engine and reaches the
pin only through the ``USRMCLK`` primitive -- so the platform's ``qspi_flash``
resource has no ``sck`` member at all, and there is no port to hand Glasgow.

Every way of faking one fails for the same underlying reason: Amaranth's Lattice
backend only knows how to build buffers for real ports. A ``SimulationPort`` is
rejected outright, and an unconstrained ``IOPort`` with a second buffer on it is
a ``DriverConflict`` -- reported at I/O-use checking rather than by ``prepare()``,
so it silently appears to work first.

``USRMCLKController`` therefore replaces the SCK buffer rather than feeding it,
by overriding the one method that builds it. That keeps all of Glasgow's timing
and framing logic intact, which is the part worth reusing.
"""

from amaranth import (Array, ClockSignal, Const, Elaboratable, Instance, Module,
                      Mux, Signal)
from amaranth.lib import io, wiring

from glasgow.gateware.ports import PortGroup
from glasgow.gateware.qspi import Controller as _GlasgowQSPIController


class USRMCLKPort(io.SimulationPort):
    """ A one-bit output port whose value is consumed by ``USRMCLK``.

    Subclasses ``SimulationPort`` because that is the one port type with plain
    signals behind it rather than a pad, and because Glasgow's
    ``SimulatableDDRBuffer`` already special-cases it -- so the DDR semantics
    come for free instead of being reimplemented.

    The buffer built for it is intercepted by ``USRMCLKDDRBuffer`` below, so no
    pad is ever allocated; the pin does not exist to allocate.
    """

    def __init__(self, *, name="usrmclk"):
        super().__init__("o", 1, name=name)
        # What the controller drives, one bit per half of the sync cycle.
        self.ddr_o = Signal(2, name=f"{name}_ddr_o")

    def elaborate_buffer(self, buffer):
        """ Build this port's buffer.

        Called by Glasgow's SimulatableDDRBuffer instead of constructing an
        io.Buffer, which the Lattice backend would reject: there is no pad here
        to build a buffer for. The value is carried out on `ddr_o` and handed
        to USRMCLK by the controller.
        """
        m = Module()
        m.d.comb += self.ddr_o.eq(buffer.o)
        return m





class QSPIFlashController(wiring.Component):
    """ Glasgow's QSPI controller, wired to the ECP5 configuration flash.

    Presents exactly Glasgow's interface -- ``i_stream``, ``o_stream`` and
    ``divisor`` -- so its documentation and its tests apply unchanged.

    Parameters
    ----------
    resource : platform resource
        The ``qspi_flash`` resource, requested with ``dir="-"``.
    offset : int
        Sample-point offset passed through to Glasgow's controller. This is the
        knob for the round-trip delay through USRMCLK, the pad, the flash's
        access time and back -- the delay that capped the single-lane
        implementation at 30 MHz SCK.
    """

    def __init__(self, *, resource, offset=0):
        self.resource = resource
        self.offset   = offset
        self.sck      = Signal()

        self._sck_port = USRMCLKPort()

        # Glasgow inverts CS itself (`cs=~ports.cs...` in its Controller),
        # because it expects an active-high port. Our platform declares cs with
        # PinsN, so the port already carries invert=True and the two cancel:
        # CS is never asserted, the flash never responds, and every read comes
        # back as zeros -- at the right speed, and identically for every sample
        # offset, which is what made it look like a sampling problem.
        cs_port = io.SingleEndedPort(resource.cs.io, invert=False,
                                     direction=resource.cs.direction)

        self._inner = _GlasgowQSPIController(
            PortGroup(cs=cs_port,
                      sck=self._sck_port,
                      io=resource.dq),
            offset=offset)

        super().__init__({
            "i_stream": wiring.In(self._inner.i_stream.signature),
            "o_stream": wiring.Out(self._inner.o_stream.signature),
            "divisor":  wiring.In(16),
        })

    def elaborate(self, platform):
        m = Module()

        m.submodules.inner = self._inner
        # Plain signal assignment rather than wiring.connect: our ports are
        # already flipped relative to the inner component, so connect() sees
        # two outputs where it wants a producer and a consumer.
        m.d.comb += [
            self._inner.i_stream.payload.eq(self.i_stream.payload),
            self._inner.i_stream.valid  .eq(self.i_stream.valid),
            self.i_stream.ready         .eq(self._inner.i_stream.ready),

            self.o_stream.payload       .eq(self._inner.o_stream.payload),
            self.o_stream.valid         .eq(self._inner.o_stream.valid),
            self._inner.o_stream.ready  .eq(self.o_stream.ready),

            self._inner.divisor         .eq(self.divisor),
        ]

        # SCK comes out of a DDR output register, so ddr_o is two bits: bit 0
        # is driven during the first half of the sync cycle, bit 1 during the
        # second. USRMCLK takes a single clock input, so a single wire has to
        # carry both halves.
        #
        # `USRMCLKPort.elaborate_buffer` replaces the buffer rather than
        # feeding it, so `ddr_o` is the DDR buffer's *input* -- unregistered.
        # Two things follow, and both are faults:
        #
        #  - Taking bit 0 alone is lossless only for divisor >= 1, where both
        #    halves hold the same value and SCK is a whole number of sync
        #    cycles. At divisor 0 a full SCK period lives inside one sync cycle
        #    and exists only as the difference between the halves, so bit 0 is
        #    a constant 0: no clock reaches the flash at all. That is the sole
        #    reason SCK = sync has never worked, and it caps SCK at sync/2.
        #  - CS and DQ go through amaranth's ECP5 `DDRBuffer`, which is an
        #    ODDRX1F and which amaranth documents as costing two pipeline
        #    stages. SCK bypassing it therefore arrives *early* relative to
        #    every signal it is supposed to clock.
        #
        # An ODDRX1F would fix both -- it serialises the two halves onto one
        # wire, and it is the same primitive with the same latency the data
        # pins get. **It cannot be used here.** nextpnr's ECP5 packer refuses
        # it outright:
        #
        #     ERROR: ODDRX1F 'controller.sck_oddr' Q output must be connected
        #     only to a top level output
        #
        # USRMCLK is not a top-level output; it is a macro standing in for a
        # pad that user logic cannot reach. So the one primitive that would
        # serialise a DDR pair onto this pin is exactly the one the pin cannot
        # have, and SCK is structurally capped at sync/2 -- one flash clock per
        # two fabric clocks -- rather than at sync.
        #
        # Reaching a given SCK therefore means building the whole design at
        # twice that rate, and the ceiling on SCK is the ceiling on the
        # design's fmax. Lifting it means generating SCK in a 2x `fast` domain
        # from a plain register, which is legal where the DDR primitive is not.
        m.d.comb += self.sck.eq(self._sck_port.ddr_o[0])

        m.submodules.usrmclk = Instance(
            "USRMCLK",
            i_USRMCLKI  = self.sck,
            i_USRMCLKTS = 0,
        )

        return m


# Fast Read Quad Output (6Bh): the command and 24-bit address go out on a
# single lane, then a dummy byte, then data returns on all four. Requires the
# QE bit in status register 2, which on this board is already set.
OPCODE_QUAD_READ    = 0x6B   # Fast Read Quad Output: address on one lane
OPCODE_QUAD_IO_READ = 0xEB   # Fast Read Quad I/O: address on four lanes
OPCODE_READ         = 0x03   # Read Data: one lane throughout, no dummy
OPCODE_FAST_READ    = 0x0B   # Fast Read: one lane, one dummy byte

# `read_mode` values. Bit 1 selects a single-lane data phase, which is why the
# ordering is what it is -- the data-phase operation is one bit test rather
# than a decode.
MODE_QUAD_OUT = 0            # 0x6B
MODE_QUAD_IO  = 1            # 0xEB
MODE_SINGLE   = 2            # 0x03
MODE_FAST     = 3            # 0x0B


class QuadFlashReader(Elaboratable):
    """ Reads a span of flash using Fast Read Quad Output.

    Four bits per clock instead of one, so at a given SCK this is four times
    the throughput of the 0x03 and 0x0B paths -- without raising SCK, which is
    where the single-lane implementation ran out of margin.

    Attributes
    ----------
    start : Signal(), in
        Strobe to begin a read from address zero.
    length : Signal(16), in
        Number of bytes to read.
    divisor : Signal(16), in
        Passed to the controller; SCK is the sync rate over this.
    data_strobe : Signal(), out
        High for one cycle per returned byte.
    data : Signal(8), out
        That byte.
    cycles : Signal(32), out
        Sync-domain cycles the transfer occupied.
    done : Signal(), out
        High once the read has completed.
    """

    def __init__(self, *, controller):
        self.ctrl = controller

        self.start       = Signal()
        self.length      = Signal(16)
        self.divisor     = Signal(16)
        # One of MODE_QUAD_OUT / MODE_QUAD_IO / MODE_SINGLE / MODE_FAST.
        #
        # 0xEB (quad I/O) sends the address on four lanes as well as the data,
        # which halves the per-transaction overhead from 40 clocks to 20. That
        # is worth little on a long streaming read -- 0.2% at 4 KiB -- and a
        # great deal on small random ones: 19% for a 32-byte cache line, 28%
        # for 16 bytes. The datasheet is explicit that it exists to allow
        # "faster random access for code execution (XIP)", which is exactly the
        # RISC-V-executing-from-flash case rather than the bulk-transfer one.
        #
        # The two single-lane modes are here to be a baseline measured by this
        # same instrument, so a quad figure is compared against a single-lane
        # one taken through the identical path rather than against a number
        # from another design.
        self.read_mode   = Signal(2)
        # The mode byte M7-0 that 0xEB sends straight after the address.
        #
        # M5-4 = (1,0) puts the part into Continuous Read: the *next* 0xEB
        # transaction omits its opcode entirely, saving 8 clocks. Anything else
        # leaves it. 0xA0 enters, 0xFF or 0x00 leaves. This is device state,
        # not controller state -- it survives an FPGA reconfiguration, and a
        # part left in Continuous Read returns garbage to a reader that starts
        # sending opcodes again, so `xip` below tracks what was actually sent.
        self.mode_byte   = Signal(8)
        # Force the next read to omit the opcode. The recovery knob for a part
        # left in Continuous Read by a bitstream that is no longer loaded.
        self.xip_force   = Signal()
        # High when the part is believed to be in Continuous Read.
        self.xip         = Signal()
        # Start address. Previously hard-wired to zero, which meant every read
        # hit the same page -- so a burst of reads measured the flash's
        # sequential path repeatedly rather than its random-access one, and
        # could not detect an address decoding fault at all.
        self.address     = Signal(24)
        self.data_strobe = Signal()
        self.data        = Signal(8)
        self.cycles      = Signal(32)
        self.done        = Signal()
        self.busy        = Signal()

    def elaborate(self, platform):
        m = Module()

        from glasgow.gateware.qspi import Operation

        # Address bytes are most-significant first, as SPI NOR expects. Clock
        # counts below are per transaction, before the data phase:
        #
        #   0x03  opcode(x1) + 3 address(x1)                         = 32
        #   0x0B  opcode(x1) + 3 address(x1) + 1 dummy(x1)           = 40
        #   0x6B  opcode(x1) + 3 address(x1) + 1 dummy(x1)           = 40
        #   0xEB  opcode(x1) + 3 address(x4) + mode(x4) + 2 dummy(x4)= 20
        #   0xEB  in Continuous Read, opcode omitted                 = 12
        #
        # An x4 beat is a quarter of the clocks of an x1 beat, which is why
        # 0xEB has more beats and less latency. The 0xEB dummy is two x4 beats
        # = 4 clocks, which is what this part specifies for that opcode; the
        # mode byte is a further 2, for 6 clocks of latency in total.
        addr_hi = self.address[16:24]
        addr_md = self.address[8:16]
        addr_lo = self.address[0:8]

        header_03 = Array([OPCODE_READ,         addr_hi, addr_md, addr_lo])
        header_0b = Array([OPCODE_FAST_READ,    addr_hi, addr_md, addr_lo,
                           0x00])
        header_6b = Array([OPCODE_QUAD_READ,    addr_hi, addr_md, addr_lo,
                           0x00])
        header_eb = Array([OPCODE_QUAD_IO_READ, addr_hi, addr_md, addr_lo,
                           self.mode_byte, 0x00, 0x00])

        header_len   = Signal(range(8))
        header_index = Signal(range(8))
        header_first = Signal(range(8))
        header_byte  = Signal(8)
        header_oper  = Signal(3)
        data_oper    = Signal(3)
        bytes_left   = Signal(16)
        received     = Signal(16)

        # Continuous Read skips index 0 -- the opcode -- and nothing else.
        skip_opcode = Signal()
        m.d.comb += [
            skip_opcode.eq((self.read_mode == MODE_QUAD_IO)
                           & (self.xip | self.xip_force)),
            header_first.eq(Mux(skip_opcode, 1, 0)),
            # Bit 1 of the mode number is exactly "single-lane data phase".
            data_oper.eq(Mux(self.read_mode[1], Operation.GetX1,
                             Operation.GetX4)),
        ]

        with m.Switch(self.read_mode):
            with m.Case(MODE_QUAD_IO):
                m.d.comb += [
                    header_len .eq(len(header_eb)),
                    header_byte.eq(header_eb[header_index]),
                    # Only the opcode goes out on a single lane; address, mode
                    # and dummy all use four.
                    header_oper.eq(Mux(header_index == 0, Operation.PutX1,
                                       Operation.PutX4)),
                ]
            with m.Case(MODE_SINGLE):
                m.d.comb += [
                    header_len .eq(len(header_03)),
                    header_byte.eq(header_03[header_index]),
                    header_oper.eq(Operation.PutX1),
                ]
            with m.Case(MODE_FAST):
                m.d.comb += [
                    header_len .eq(len(header_0b)),
                    header_byte.eq(header_0b[header_index]),
                    header_oper.eq(Operation.PutX1),
                ]
            with m.Default():
                m.d.comb += [
                    header_len .eq(len(header_6b)),
                    header_byte.eq(header_6b[header_index]),
                    header_oper.eq(Operation.PutX1),
                ]

        # Latched at IDLE, not passed through. `divisor` is a host register and
        # Glasgow's enframer reads it live, comparing it against a free-running
        # timer -- so a write that lands mid-transaction changes the clock
        # period between one byte and the next and the flash sees a clock it
        # was never told about. Same rule the I2C mux follows for its bus
        # select: a bus parameter changes where the bus is idle.
        divisor_held = Signal(16)
        m.d.comb += self.ctrl.divisor.eq(divisor_held)

        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.start):
                    m.d.sync += [
                        header_index.eq(header_first),
                        divisor_held.eq(self.divisor),
                        bytes_left  .eq(self.length),
                        received    .eq(0),
                        self.cycles .eq(0),
                        self.done   .eq(0),
                    ]
                    m.next = "HEADER"

            with m.State("HEADER"):
                m.d.comb += [
                    self.busy                   .eq(1),
                    self.ctrl.i_stream.valid    .eq(1),
                    self.ctrl.i_stream.payload.chip.eq(1),
                    self.ctrl.i_stream.payload.oper.eq(header_oper),
                    self.ctrl.i_stream.payload.data.eq(header_byte),
                ]
                m.d.sync += self.cycles.eq(self.cycles + 1)
                with m.If(self.ctrl.i_stream.ready):
                    with m.If(header_index == header_len - 1):
                        m.next = "PAYLOAD"
                    with m.Else():
                        m.d.sync += header_index.eq(header_index + 1)

            with m.State("PAYLOAD"):
                m.d.comb += [
                    self.busy                   .eq(1),
                    self.ctrl.i_stream.valid    .eq(bytes_left != 0),
                    # chip 0 deselects, ending the transaction on the last byte
                    self.ctrl.i_stream.payload.chip.eq(
                        Mux(bytes_left == 0, 0, 1)),
                    self.ctrl.i_stream.payload.oper.eq(data_oper),
                    self.ctrl.i_stream.payload.data.eq(0),
                    self.ctrl.o_stream.ready    .eq(1),
                ]
                m.d.sync += self.cycles.eq(self.cycles + 1)

                with m.If(self.ctrl.i_stream.valid & self.ctrl.i_stream.ready):
                    m.d.sync += bytes_left.eq(bytes_left - 1)

                with m.If(self.ctrl.o_stream.valid):
                    m.d.comb += [
                        self.data_strobe.eq(1),
                        self.data.eq(self.ctrl.o_stream.payload.data),
                    ]
                    m.d.sync += received.eq(received + 1)

                # Wait until every requested byte has actually come back. The
                # controller's pipeline is several cycles deep, so finishing
                # when the last byte is *requested* drops the tail -- observed
                # in simulation as 7 bytes returned for a length of 8.
                with m.If(received == self.length):
                    m.next = "DESELECT"

            with m.State("DESELECT"):
                # Explicitly deselect the chip. The `chip` field of the last
                # payload beat cannot do this: by then bytes_left is 0 so
                # i_stream.valid is low and the frame is never sent, leaving CS
                # asserted and the flash mid-stream.
                #
                # A single read still succeeds -- the first transaction after
                # configuration is clean -- so this only shows up on the second
                # read, which then starts partway through the flash's output
                # and returns data from the wrong address. Observed as byte-
                # exact on the power-on run and corrupt on every re-trigger,
                # with the same divisor and identical cycle count.
                m.d.comb += [
                    self.busy                      .eq(1),
                    self.ctrl.i_stream.valid       .eq(1),
                    self.ctrl.i_stream.payload.chip.eq(0),
                    self.ctrl.i_stream.payload.oper.eq(Operation.Idle),
                    self.ctrl.i_stream.payload.data.eq(0),
                    self.ctrl.o_stream.ready       .eq(1),
                ]
                with m.If(self.ctrl.i_stream.ready):
                    m.d.sync += self.done.eq(1)
                    # Track what the part was actually told, not what was
                    # wanted. A read whose mode byte carried M5-4 = (1,0) left
                    # it in Continuous Read; any other read, in any other mode,
                    # took it out. Deriving this from the transaction that just
                    # happened is what makes leaving Continuous Read a matter
                    # of doing one ordinary read rather than of remembering to
                    # clear a flag.
                    m.d.sync += self.xip.eq(
                        (self.read_mode == MODE_QUAD_IO)
                        & (self.mode_byte[4:6] == 0b10))
                    m.next = "IDLE"

        return m
