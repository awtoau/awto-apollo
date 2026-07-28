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

from amaranth import Array, Elaboratable, Instance, Module, Mux, Signal
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
        # second. USRMCLK takes a single clock input, so only bit 0 is
        # forwarded.
        #
        # That is lossless for divisor >= 1, where both halves always hold the
        # same value and SCK is a whole number of sync cycles. It is NOT
        # lossless at divisor 0, where a full clock period lives inside one
        # sync cycle and exists only as the difference between the halves:
        # keeping bit 0 alone leaves a constant 0, the flash never sees a
        # clock, and reads return zeros at every sample offset. This is the
        # sole reason 60 MHz SCK fails; the part is rated to 104 MHz.
        #
        # To lift it, serialise both halves through an ODDRX1F clocked at 2x
        # and drive USRMCLK from its output -- the construction LUNA already
        # uses for the HyperRAM clock (i_D0/i_D1 -> o_Q).
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
OPCODE_QUAD_READ = 0x6B


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
        self.data_strobe = Signal()
        self.data        = Signal(8)
        self.cycles      = Signal(32)
        self.done        = Signal()
        self.busy        = Signal()

    def elaborate(self, platform):
        m = Module()

        from glasgow.gateware.qspi import Operation

        # Opcode, three address bytes and one dummy byte, all single-lane.
        header       = Array([OPCODE_QUAD_READ, 0x00, 0x00, 0x00, 0x00])
        header_index = Signal(range(len(header)))
        bytes_left   = Signal(16)
        received     = Signal(16)

        m.d.comb += self.ctrl.divisor.eq(self.divisor)

        with m.FSM():
            with m.State("IDLE"):
                with m.If(self.start):
                    m.d.sync += [
                        header_index.eq(0),
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
                    self.ctrl.i_stream.payload.oper.eq(Operation.PutX1),
                    self.ctrl.i_stream.payload.data.eq(header[header_index]),
                ]
                m.d.sync += self.cycles.eq(self.cycles + 1)
                with m.If(self.ctrl.i_stream.ready):
                    with m.If(header_index == len(header) - 1):
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
                    self.ctrl.i_stream.payload.oper.eq(Operation.GetX4),
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
                    m.d.sync += self.done.eq(1)
                    m.next = "IDLE"

        return m
