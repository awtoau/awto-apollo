#
# This file is part of Apollo.
#
# Copyright (c) 2020-2026 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" A clock generator that runs `sync` at an arbitrary frequency.

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
import re
import shutil
import subprocess
from pathlib import Path

from amaranth import (ClockDomain, ClockSignal, Elaboratable, Instance, Module,
                      ResetSignal, Signal)


# Project Trellis's PLL calculator. Asking it is the whole point of this
# module: the ECP5 PLL has four coupled parameters (CLKI_DIV, CLKFB_DIV,
# CLKOP_DIV and the VCO they imply) and picking them by hand is how this went
# wrong twice -- once blaming the output dividers, once "fixing" the VCO to a
# value that matched the symptoms while being wrong about the cause.
#
# ecppll ships with the OSS CAD Suite and chooses a different VCO per target
# frequency: 480 MHz for 240, 576 for 192, 640 for 160. Any implementation
# that assumes one fixed VCO can only reach that VCO's integer divisions,
# which is why an earlier version could produce 240, 160 and 120 MHz but
# nothing in between.
# Located rather than assumed to be on PATH: ecppll lives in the OSS CAD Suite,
# which is usually only on PATH inside its own environment, so a build script
# that sources that environment finds it while an ordinary import does not.
def _find_ecppll():
    found = shutil.which("ecppll")
    if found:
        return found
    for candidate in (Path.home() / "opt/oss-cad-suite/bin/ecppll",):
        if candidate.exists():
            return str(candidate)
    return "ecppll"


ECPPLL = _find_ecppll()


# How far `usb` may sit from 60 MHz before the build is refused.
#
# The ULPI PHY is fed a fixed 60 MHz and has no tolerance to speak of -- this is
# a source-synchronous parallel interface, not a UART with a resynchronising
# start bit, so there is no mechanism to absorb a frequency error at all. 0.5%
# is therefore already permissive; it exists to allow floating-point noise on an
# exact division rather than to allow real error.
#
# Measured consequence of exceeding it: a 90 MHz sync build (usb 63.000 MHz,
# +5%) placed, packed and configured cleanly and then never appeared on the USB
# bus, while a 100 MHz build -- a *higher* CPU clock, but usb exactly 60.000 --
# enumerated at once. The failure mode is a silently dead device, which is why
# this refuses at build time instead of warning.
USB_CLOCK_TOLERANCE_PCT = 0.5


def _ecppll(sync_mhz, input_mhz=60.0):
    """ Ask ecppll for a configuration, returning its parameters as a dict.

    Raises if ecppll is unavailable: guessing the parameters is precisely the
    failure mode this exists to avoid, so a silent fallback would be worse
    than not building.
    """
    result = subprocess.run(
        [ECPPLL, "-i", str(input_mhz), "-o", str(sync_mhz), "-f", "/dev/stdout"],
        capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"{ECPPLL} could not generate a {sync_mhz} MHz clock from "
            f"{input_mhz} MHz: {result.stderr.strip()}")

    params = {}
    for line in result.stdout.splitlines():
        line = line.strip()
        match = re.match(r"\.(\w+)\(([^)]*)\),?", line)
        if match:
            name, value = match.group(1), match.group(2).strip('"')
            params[name] = int(value) if value.lstrip("-").isdigit() else value
        match = re.match(r"clkout0 frequency: ([\d.]+) MHz", line)
        if match:
            params["_actual_mhz"] = float(match.group(1))
        match = re.match(r"VCO frequency: ([\d.]+)", line)
        if match:
            params["_vco_mhz"] = float(match.group(1))
    return params


class VariableClockDomainGenerator(Elaboratable):
    """ Generates `sync` at an arbitrary frequency, with `usb` fixed at 60 MHz.

    Parameters
    ----------
    sync_mhz : float
        Desired sync frequency. ecppll picks the dividers and the VCO.

    Attributes
    ----------
    actual_sync_mhz : float
        What ecppll says will actually be produced. Report this rather than the
        request: rates derived from a requested value that was not achieved are
        wrong by exactly the rounding error.
    """

    def __init__(self, *, sync_mhz=60.0, input_mhz=60.0):
        self.requested_sync_mhz = sync_mhz
        self._params = _ecppll(sync_mhz, input_mhz)
        self.actual_sync_mhz = self._params.get("_actual_mhz", sync_mhz)
        self.vco_mhz = self._params.get("_vco_mhz")

        if abs(self.actual_sync_mhz - sync_mhz) > 0.01:
            logging.info("sync %.1f MHz requested, %.1f MHz produced "
                         "(VCO %.0f)",
                         sync_mhz, self.actual_sync_mhz, self.vco_mhz or 0)

        # Checked here rather than in elaborate() so it fires on construction,
        # before a platform is involved -- the answer depends only on the PLL
        # parameters, and a caller sweeping frequencies should learn which ones
        # are legal without needing to build.
        self.usb_div = int(round(self.vco_mhz / 60.0))
        self.actual_usb_mhz = self.vco_mhz / self.usb_div
        self.usb_error_pct = 100.0 * (self.actual_usb_mhz - 60.0) / 60.0
        if abs(self.usb_error_pct) > USB_CLOCK_TOLERANCE_PCT:
            raise ValueError(
                f"sync {sync_mhz:g} MHz gives VCO {self.vco_mhz:g} MHz, so "
                f"usb = {self.vco_mhz:g}/{self.usb_div} = "
                f"{self.actual_usb_mhz:.3f} MHz ({self.usb_error_pct:+.2f}%). "
                f"The ULPI PHY needs 60 MHz and the design will not enumerate. "
                f"Usable frequencies are those whose VCO is a multiple of 60 "
                f"-- 60, 100 and 120 MHz are the only ones in 60..130.")

    def elaborate(self, platform):
        m = Module()

        m.domains.sync = ClockDomain()
        m.domains.usb  = ClockDomain()

        clk_sync = Signal()
        clk_usb  = Signal()
        locked   = Signal()

        params = self._params
        input_clock = platform.request(platform.default_clk).i

        # sync comes from CLKOP, which is also the feedback path, so ecppll's
        # CLKOP_DIV/CLKFB_DIV pair is used exactly as given. usb comes from a
        # secondary output divided to 60 MHz.
        #
        # That division is an INTEGER, so `usb` only lands on exactly 60 MHz
        # when the VCO is a whole multiple of 60. It very often is not, and the
        # error is silent at build time and fatal on the board:
        #
        #     sync  90 MHz -> VCO 630 -> div 10 -> usb 63.000 MHz  (+5.00%)
        #     sync 110 MHz -> VCO 550 -> div  9 -> usb 61.111 MHz  (+1.85%)
        #     sync  80 MHz -> VCO 560 -> div  9 -> usb 62.222 MHz  (+3.70%)
        #     sync 100 MHz -> VCO 600 -> div 10 -> usb 60.000 MHz  (exact)
        #
        # The ULPI PHY requires 60 MHz. At 63 MHz the design does not enumerate
        # at all -- measured, not inferred: a 90 MHz build placed and packed
        # cleanly, configured onto the board, and never appeared on the bus,
        # while a 100 MHz build (a *higher* sync clock, but an exact 60 MHz
        # usb) enumerated immediately. So a failure here looks exactly like
        # "the CPU is too fast" while being nothing of the sort, and it sent an
        # earlier investigation looking for a timing ceiling that was not
        # there.
        #
        # Refusing to build is the right response. A design whose USB cannot
        # work is not a design worth loading, and the alternative -- a dead
        # device with no explanation -- costs far more than a build error.
        # Validated in __init__, which raises rather than letting a design with
        # an unusable USB clock reach the board.
        usb_div = self.usb_div

        m.submodules.pll = Instance(
            "EHXPLLL",

            i_CLKI=input_clock,
            i_CLKFB=clk_sync,
            i_PHASESEL0=0, i_PHASESEL1=0,
            i_PHASEDIR=1, i_PHASESTEP=1, i_PHASELOADREG=1,
            i_STDBY=0, i_PLLWAKESYNC=0, i_RST=0, i_ENCLKOP=0,

            o_CLKOP=clk_sync,
            o_CLKOS=clk_usb,
            o_LOCK=locked,

            p_PLLRST_ENA="DISABLED",
            p_INTFB_WAKE="DISABLED",
            p_STDBY_ENABLE="DISABLED",
            p_DPHASE_SOURCE="DISABLED",
            p_OUTDIVIDER_MUXA="DIVA",
            p_OUTDIVIDER_MUXB="DIVB",
            p_OUTDIVIDER_MUXC="DIVC",
            p_OUTDIVIDER_MUXD="DIVD",

            p_CLKI_DIV=params["CLKI_DIV"],
            p_CLKFB_DIV=params["CLKFB_DIV"],
            p_FEEDBK_PATH="CLKOP",

            p_CLKOP_ENABLE="ENABLED",
            p_CLKOP_DIV=params["CLKOP_DIV"],
            p_CLKOP_CPHASE=params["CLKOP_DIV"] - 1,
            p_CLKOP_FPHASE=0,

            p_CLKOS_ENABLE="ENABLED",
            p_CLKOS_DIV=usb_div,
            p_CLKOS_CPHASE=usb_div - 1,
            p_CLKOS_FPHASE=0,

            a_ICP_CURRENT="12",
            a_LPF_RESISTOR="8",
            a_MFG_ENABLE_FILTEROPAMP="1",
            a_MFG_GMCREF_SEL="2",
        )

        m.d.comb += [
            ClockSignal("sync").eq(clk_sync),
            ClockSignal("usb") .eq(clk_usb),

            # Hold both domains in reset until the PLL locks: a domain clocked
            # by an unlocked PLL sees a frequency that drifts as it settles.
            ResetSignal("sync").eq(~locked),
            ResetSignal("usb") .eq(~locked),
        ]

        return m
