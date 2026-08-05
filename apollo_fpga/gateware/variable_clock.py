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


# ECP5 EHXPLLL limits, from the datasheet. The VCO must sit in this window; each of the
# four outputs then divides it independently.
VCO_MIN_MHZ = 400.0
VCO_MAX_MHZ = 800.0
MAX_DIV = 128


def _solve_both(sync_mhz, usb_mhz=60.0, input_mhz=60.0, tolerance=0.001,
                fast_ratio=None):
    """Find PLL dividers giving `sync_mhz` AND `usb_mhz` exactly, or None.

    ecppll optimises for its primary output and lets the secondary land wherever the
    resulting VCO puts it. That is why asking it for 80/60 returns usb at 62.222 MHz --
    3.7% out, so the PHY does not enumerate. It is a property of the heuristic, not of
    the hardware: the outputs have independent dividers, and choosing the VCO to serve
    both is a search ecppll simply does not run.

    Searching it directly finds exact solutions for 80, 90, 100, 110 and 120 MHz, all
    with usb at 60.000. Those frequencies were previously reported as unreachable.

    `fast_ratio`, when given, additionally requires an output at `fast_ratio * sync_mhz`
    -- the `fast` domain HyperRAM's PHY clocks its DDR output registers from. Since every
    output divides the same VCO, that means CLKOP_DIV must itself be divisible by the
    ratio, so solutions exist only where it is. Asking for it and silently getting a
    slightly-wrong `fast` would be the same class of failure as the usb one below, so the
    constraint is imposed in the search rather than checked afterwards.

    Returns (vco_mhz, clki_div, clkfb_div, clkop_div, clkos_div, clkos2_div); the last is
    None when `fast_ratio` is None.
    """
    # FEEDBK_PATH is "CLKOP", so the feedback divider counts OUTPUT periods, not VCO
    # periods: VCO = input * CLKFB_DIV * CLKOP_DIV / CLKI_DIV. Treating CLKFB_DIV as a
    # plain VCO multiplier is exactly the mistake this file's header warns about -- an
    # earlier attempt produced clocks at twice the requested rate and was abandoned as
    # "the dividers do not behave as documented". Verified against ecppll, which returns
    # CLKI_DIV=2 CLKFB_DIV=3 CLKOP_DIV=7 for 90 MHz: 60 * 3 * 7 / 2 = 630 MHz VCO.
    for clki_div in range(1, 8):
        for clkop_div in range(1, MAX_DIV + 1):
            for clkfb_div in range(1, 81):
                vco = input_mhz * clkfb_div * clkop_div / clki_div
                if not VCO_MIN_MHZ <= vco <= VCO_MAX_MHZ:
                    continue
                if abs(vco / clkop_div - sync_mhz) > tolerance:
                    continue

                # `fast` divides the same VCO, so it exists only when CLKOP_DIV divides
                # evenly by the ratio. Rejecting here rather than after the usb match
                # keeps the search from returning a pair that cannot carry all three.
                clkos2_div = None
                if fast_ratio is not None:
                    if clkop_div % fast_ratio:
                        continue
                    clkos2_div = clkop_div // fast_ratio

                for clkos_div in range(1, MAX_DIV + 1):
                    if abs(vco / clkos_div - usb_mhz) <= tolerance:
                        return (vco, clki_div, clkfb_div, clkop_div, clkos_div,
                                clkos2_div)
    return None


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

    def __init__(self, *, sync_mhz=60.0, input_mhz=60.0, with_fast=False,
                 fast_ratio=2):
        self.requested_sync_mhz = sync_mhz
        self.with_fast = with_fast
        self.fast_ratio = fast_ratio
        self.fast_div = None
        self.actual_fast_mhz = None

        # Solve for both outputs first. ecppll only optimises the primary, which is what
        # made 80, 90 and 110 MHz look unreachable -- they are not, and this finds them.
        solved = _solve_both(sync_mhz, 60.0, input_mhz,
                             fast_ratio=fast_ratio if with_fast else None)
        if solved:
            vco, clki_div, clkfb_div, clkop_div, clkos_div, clkos2_div = solved
            self._params = {"CLKI_DIV": clki_div, "CLKFB_DIV": clkfb_div,
                            "CLKOP_DIV": clkop_div, "_vco_mhz": vco,
                            "_actual_mhz": vco / clkop_div}
            self._solved_usb_div = clkos_div
            if with_fast:
                self.fast_div = clkos2_div
                self.actual_fast_mhz = vco / clkos2_div
        elif with_fast:
            # No fallback here. ecppll cannot express a three-output constraint, so
            # accepting its answer would give a `fast` domain at whatever the VCO
            # happened to allow -- and HyperRAM clocked from a wrong `fast` corrupts
            # data rather than failing, which is far worse than not building.
            raise ValueError(
                f"no PLL configuration gives sync {sync_mhz:g} MHz, usb 60 MHz and "
                f"fast {fast_ratio * sync_mhz:g} MHz together. `fast` divides the same "
                f"VCO as sync, so CLKOP_DIV must be a multiple of {fast_ratio}; try 60 "
                f"or 100 MHz.")
        else:
            # No exact pair exists; fall back to ecppll and let the usb check below
            # decide whether the result is usable.
            self._params = _ecppll(sync_mhz, input_mhz)
            self._solved_usb_div = None

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
        self.usb_div = (self._solved_usb_div if self._solved_usb_div
                        else int(round(self.vco_mhz / 60.0)))
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
        clk_fast = Signal()
        locked   = Signal()

        # `fast` is HyperRAM's DDR clock, at fast_ratio x sync. Only created on request:
        # an unused domain still consumes a PLL output and a global buffer.
        if self.with_fast:
            m.domains.fast = ClockDomain()

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
            o_CLKOS2=clk_fast,
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

            p_CLKOS2_ENABLE="ENABLED" if self.with_fast else "DISABLED",
            p_CLKOS2_DIV=self.fast_div or 1,
            p_CLKOS2_CPHASE=(self.fast_div or 1) - 1,
            p_CLKOS2_FPHASE=0,

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

        if self.with_fast:
            m.d.comb += [
                ClockSignal("fast").eq(clk_fast),
                ResetSignal("fast").eq(~locked),
            ]

        return m
