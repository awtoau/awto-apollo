#
# This file is part of Apollo.
#
# Copyright (c) 2020-2026 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" A clock generator that runs `sync` at any 480/N MHz.

``LunaECP5DomainGenerator`` always clocks `sync` at 60 MHz and offers only
60/120/240 elsewhere, which forces a speed ladder to step in factors of two and
leaves wide untested gaps -- the flash work could reach 60 MHz SCK and 120, with
nothing in between, while the part is rated to 104.

Nothing in the hardware requires those values. The PLL runs a 480 MHz VCO and
each output divides it, so 160, 96, 80, 68.6 and the rest are all available.
This is LUNA's configuration with one divider parameterised.

An earlier attempt produced clocks at exactly twice the requested rate and was
abandoned as "the dividers do not behave as documented". They behave fine: the
feedback divider had been copied from LUNA without adjusting it, which doubled
the VCO. Check any change here against ``ecppll`` before trusting it.
"""

import logging

from amaranth import (ClockDomain, ClockSignal, Elaboratable, Instance, Module,
                      ResetSignal, Signal)


# The VCO frequency in MHz, and the feedback divider that produces it.
#
# These are not free parameters. With FEEDBK_PATH="CLKOP" the loop locks when
#
#     CLKOP = input * CLKFB_DIV        and        VCO = CLKOP * CLKOP_DIV
#
# so CLKFB_DIV must track CLKOP_DIV. For a 240 MHz CLKOP from a 60 MHz input:
# CLKFB_DIV = 240/60 = 4, and VCO = 240 * 2 = 480.
#
# Verified against Project Trellis's own calculator rather than derived here:
#
#     ecppll -i 60 -o 240 --clkout1 160
#     -> VCO frequency: 480, CLKOP_DIV 2, CLKFB_DIV 4, CLKOS_DIV 3
#
# Getting this wrong is quiet and expensive. LUNA uses CLKFB_DIV=8 with
# CLKOP_DIV=2, which doubles the VCO to 960; carrying that value over here
# while computing dividers against 480 produced every clock at exactly twice
# the requested rate. Two rounds of debugging blamed the dividers, and a
# measurement-based "correction" to VCO_MHZ=960 fitted the observations while
# being wrong about the cause.
VCO_MHZ = 480.0
CLKOP_DIV = 2
CLKFB_DIV = 4

# ECP5 output dividers are integers; 1 is the VCO itself, which is not a usable
# fabric clock on this part.
MIN_DIV = 2
MAX_DIV = 128


def achievable_frequencies(min_mhz=20.0, max_mhz=250.0):
    """ Every sync frequency this can produce, as (frequency_mhz, divider). """
    return [(VCO_MHZ / div, div)
            for div in range(MIN_DIV, MAX_DIV + 1)
            if min_mhz <= VCO_MHZ / div <= max_mhz]


def nearest_frequency(requested_mhz):
    """ The achievable frequency closest to `requested_mhz`, and its divider.

    Snapping rather than raising: the caller is usually a ladder script that
    wants to know what it actually got, and every result is reported with the
    real frequency rather than the requested one.
    """
    return min(achievable_frequencies(min_mhz=1.0, max_mhz=VCO_MHZ),
               key=lambda pair: abs(pair[0] - requested_mhz))


class VariableClockDomainGenerator(Elaboratable):
    """ Generates `sync` at an arbitrary 480/N MHz, with `usb` fixed at 60.

    Parameters
    ----------
    sync_mhz : float
        Desired sync frequency. Snapped to the nearest achievable value; read
        ``actual_sync_mhz`` for what was produced.

    Attributes
    ----------
    actual_sync_mhz : float
        The frequency actually generated. Report this rather than the request:
        every rate derived from it would otherwise be wrong by the snapping
        error.
    """

    def __init__(self, *, sync_mhz=60.0):
        self.requested_sync_mhz = sync_mhz
        self.actual_sync_mhz, self._sync_div = nearest_frequency(sync_mhz)

        if abs(self.actual_sync_mhz - sync_mhz) > 0.01:
            logging.info("sync %.1f MHz requested, %.1f MHz produced "
                         "(VCO %.0f / %d)",
                         sync_mhz, self.actual_sync_mhz, VCO_MHZ,
                         self._sync_div)

    def elaborate(self, platform):
        m = Module()

        m.domains.sync = ClockDomain()
        m.domains.fast = ClockDomain()
        m.domains.usb  = ClockDomain()

        clk_240  = Signal()   # CLKOP, the feedback path
        clk_sync = Signal()   # CLKOS, the parameterised output
        clk_60   = Signal()   # CLKOS2, for USB
        locked   = Signal()

        input_clock = platform.request(platform.default_clk).i

        m.submodules.pll = Instance(
            "EHXPLLL",

            i_CLKI=input_clock,
            i_CLKFB=clk_240,
            i_PHASESEL0=0, i_PHASESEL1=0,
            i_PHASEDIR=1, i_PHASESTEP=1, i_PHASELOADREG=1,
            i_STDBY=0, i_PLLWAKESYNC=0, i_RST=0, i_ENCLKOP=0,

            o_CLKOP=clk_240,
            o_CLKOS=clk_sync,
            o_CLKOS2=clk_60,
            o_LOCK=locked,

            p_PLLRST_ENA="DISABLED",
            p_INTFB_WAKE="DISABLED",
            p_STDBY_ENABLE="DISABLED",
            p_DPHASE_SOURCE="DISABLED",
            p_OUTDIVIDER_MUXA="DIVA",
            p_OUTDIVIDER_MUXB="DIVB",
            p_OUTDIVIDER_MUXC="DIVC",
            p_OUTDIVIDER_MUXD="DIVD",

            p_CLKI_DIV=1,
            p_CLKFB_DIV=CLKFB_DIV,
            p_FEEDBK_PATH="CLKOP",

            # CLKOP closes the feedback loop, so its divider sets the VCO and
            # must stay at LUNA's value.
            p_CLKOP_ENABLE="ENABLED",
            p_CLKOP_DIV=CLKOP_DIV,
            p_CLKOP_CPHASE=1,
            p_CLKOP_FPHASE=0,
            p_CLKOP_TRIM_DELAY="0",
            p_CLKOP_TRIM_POL="FALLING",

            # The one parameter that varies.
            p_CLKOS_ENABLE="ENABLED",
            p_CLKOS_DIV=self._sync_div,
            p_CLKOS_CPHASE=self._sync_div - 1,
            p_CLKOS_FPHASE=0,
            p_CLKOS_TRIM_DELAY="0",
            p_CLKOS_TRIM_POL="FALLING",

            p_CLKOS2_ENABLE="ENABLED",
            p_CLKOS2_DIV=8,
            p_CLKOS2_CPHASE=7,
            p_CLKOS2_FPHASE=0,

            # Left DISABLED, as LUNA has it. Enabling this with CLKOS3_DIV=1
            # is what produced a 480 MHz sync domain in the first attempt.
            p_CLKOS3_ENABLE="DISABLED",
            p_CLKOS3_DIV=1,
            p_CLKOS3_CPHASE=0,
            p_CLKOS3_FPHASE=0,

            a_ICP_CURRENT="12",
            a_LPF_RESISTOR="8",
            a_MFG_ENABLE_FILTEROPAMP="1",
            a_MFG_GMCREF_SEL="2",
        )

        m.d.comb += [
            ClockSignal("sync").eq(clk_sync),
            # `fast` is the 240 MHz output. Nothing here uses it as a data
            # clock; it exists because LUNA's PHYs expect the domain to be
            # present.
            ClockSignal("fast").eq(clk_240),
            ClockSignal("usb") .eq(clk_60),

            # Hold every domain in reset until the PLL locks: a domain clocked
            # by an unlocked PLL sees a frequency that drifts as it settles.
            ResetSignal("sync").eq(~locked),
            ResetSignal("fast").eq(~locked),
            ResetSignal("usb") .eq(~locked),
        ]

        return m
