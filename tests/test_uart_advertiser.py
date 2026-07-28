#
# This file is part of Apollo.
# SPDX-License-Identifier: BSD-3-Clause
#
"""
Simulation tests for ApolloUARTAdvertiser.

Decodes the transmitted waveform back into bytes rather than inspecting
internal state, so these test what Apollo's SERCOM1 receiver would actually
see on FPGA_ADV. A bug that only manifests on the wire -- wrong bit order,
a truncated stop bit, a dropped byte -- is exactly what internal-state
assertions miss.

That is not hypothetical: the first version of this gateware loaded the shift
register without waiting for a bit boundary, and transmitted C1 51 A5 instead
of C1 14 01 A5. It looked correct by inspection.

No hardware required.
"""

import unittest

from amaranth.hdl import Signal, Module
from amaranth.sim import Simulator

from apollo_fpga.gateware.advertiser import ApolloUARTAdvertiser


# A 10 MHz sim clock at 1 Mbaud gives a 10-cycle bit period: fast to simulate
# and an exact divisor, so decoding needs no tolerance handling.
SIM_CLOCK_HZ = 10e6
SIM_BAUD = 1e6
BIT_CYCLES = int(SIM_CLOCK_HZ // SIM_BAUD)


class _Pad:
    """Minimal stand-in for a platform pad; idle high, as a UART line rests."""
    def __init__(self):
        self.o = Signal(init=1)


def _capture(stop=False, cycles=4000, interval_ms=0.05):
    """Run the advertiser and return the sampled line state per clock cycle."""
    pad = _Pad()
    dut = ApolloUARTAdvertiser(pad=pad, clk_freq_hz=SIM_CLOCK_HZ,
                              baud=SIM_BAUD, interval_ms=interval_ms)

    m = Module()
    m.submodules.dut = dut

    sim = Simulator(m)
    sim.add_clock(1 / SIM_CLOCK_HZ)

    samples = []

    async def testbench(ctx):
        if stop:
            ctx.set(dut.stop, 1)
        for _ in range(cycles):
            await ctx.tick()
            samples.append(ctx.get(pad.o))

    sim.add_testbench(testbench)
    sim.run()
    return samples


def _decode(samples):
    """Decode 8-N-1 bytes from a sampled line, the way a UART receiver would."""
    decoded = []
    index = 0
    while index < len(samples) - BIT_CYCLES * 10:
        # A falling edge on an idle-high line is a start bit.
        if samples[index] == 1 and samples[index + 1] == 0:
            start = index + 1
            byte = 0
            for bit in range(8):
                # Sample mid-bit, which is what a real receiver does.
                position = start + BIT_CYCLES // 2 + BIT_CYCLES * (bit + 1)
                byte |= samples[position] << bit
            decoded.append(byte)
            index = start + BIT_CYCLES * 10
        else:
            index += 1
    return decoded


class UARTAdvertiserTest(unittest.TestCase):

    def test_transmits_the_heartbeat_pattern(self):
        """The decoded bytes must be exactly the pattern Apollo matches."""
        decoded = _decode(_capture())
        self.assertGreaterEqual(
            len(decoded), len(ApolloUARTAdvertiser.PATTERN),
            "no complete frame was transmitted")
        self.assertEqual(
            decoded[:len(ApolloUARTAdvertiser.PATTERN)],
            ApolloUARTAdvertiser.PATTERN,
            "transmitted bytes do not match HEARTBEAT_PATTERN in fpga_adv.c")

    def test_repeats_the_frame(self):
        """Apollo times out after 300 ms, so frames must keep coming."""
        decoded = _decode(_capture())
        pattern = ApolloUARTAdvertiser.PATTERN
        self.assertGreaterEqual(
            len(decoded), len(pattern) * 2,
            "expected at least two frames in the capture window")
        self.assertEqual(decoded[len(pattern):len(pattern) * 2], pattern,
                         "the second frame differs from the first")

    def test_line_idles_high_between_frames(self):
        """A UART line rests high; without that there is no falling start bit.

        Checks for an idle run longer than a whole frame (10 bit periods),
        which can only occur in the gap between frames -- inside a frame the
        line cannot stay high that long. Sampling the tail of the capture would
        not do: it lands mid-transmission depending on where the window ends.
        """
        # A longer interval than the other tests use, so the gap is unambiguous.
        samples = _capture(cycles=3000, interval_ms=0.2)
        self.assertEqual(samples[0], 1, "line did not start idle-high")

        longest = current = 0
        for value in samples:
            current = current + 1 if value == 1 else 0
            longest = max(longest, current)

        self.assertGreater(
            longest, BIT_CYCLES * 10,
            "no idle period longer than one frame -- the line is not resting "
            "high between frames")

    def test_stop_suppresses_transmission(self):
        """Asserting stop must silence the advertiser.

        This is how the FPGA hands the port back: Apollo sees no frames, its
        heartbeat goes stale, and it reclaims the port.
        """
        samples = _capture(stop=True)
        self.assertTrue(all(s == 1 for s in samples),
                        "line was driven low while stop was asserted")
        self.assertEqual(_decode(samples), [],
                         "frames were transmitted despite stop")


if __name__ == "__main__":
    unittest.main()
