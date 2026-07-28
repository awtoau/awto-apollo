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

from amaranth import Elaboratable, Instance, Module, Signal
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
        self._inner = _GlasgowQSPIController(
            PortGroup(cs=resource.cs,
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

        # The controller drives SCK as two bits, one per half of the sync
        # cycle, since it clocks at DDR rate internally. USRMCLK takes a
        # single-ended clock, so only the first half is forwarded; the
        # effective SCK is the sync rate divided by the divisor.
        m.d.comb += self.sck.eq(self._sck_port.ddr_o[0])

        m.submodules.usrmclk = Instance(
            "USRMCLK",
            i_USRMCLKI  = self.sck,
            i_USRMCLKTS = 0,
        )

        return m
