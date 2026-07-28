#
# This file is part of Apollo.
# SPDX-License-Identifier: BSD-3-Clause
#
"""
Simulation tests for the FPGA_ADV command responder.

Drives real UART waveforms in and decodes real UART waveforms out, rather than
inspecting internal state. These test what Apollo would actually see on the
wire, which is the only thing that matters -- a bug that only manifests in the
transmitted signal is exactly what state assertions miss.

That is not hypothetical. An earlier version of the advertiser gateware loaded
its shift register without waiting for a bit boundary and transmitted
C1 51 A5 where C1 14 01 A5 was intended. It looked correct by inspection.

No hardware required.
"""

import unittest

from amaranth.hdl import Module, Signal
from amaranth.sim import Simulator

from apollo_fpga.gateware.sideband import (
    SidebandResponder, CMD_PING, CMD_STATUS, CMD_POWER,
    STATUS_OK, STATUS_HEARTBEAT, STATUS_EVENTS, PROTOCOL_VERSION,
    CMD_LED_BASE,
)


# A small clock and an exact divisor keep simulations quick and make decoding
# arithmetic rather than tolerance-based.
CLK_HZ = 1e6
BAUD = 62500
DIVISOR = int(CLK_HZ // BAUD)   # 16 -- also the oversampling factor


def crc8(data):
    """CRC-8/ATM, matching the gateware: poly 0x07, init 0x00."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class SidebandTest(unittest.TestCase):

    def _run(self, command, power_data=0, **flags):
        """Send one command; return the decoded response bytes."""
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
        m = Module()
        m.submodules.dut = dut

        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)

        captured = []

        async def drive(ctx):
            ctx.set(dut.power_data, power_data)
            for name, value in flags.items():
                ctx.set(getattr(dut, name), value)

            # Capture from the first cycle, not from after the command is sent.
            # The responder begins replying as soon as the command's stop bit
            # lands, so starting the capture later lands mid-transmission and
            # the decoder never finds an idle-to-start edge to lock onto.
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 2):
                await ctx.tick()
                captured.append(ctx.get(dut.tx))

            # Transmit the command byte: start, 8 data LSB first, stop.
            for bit in [0] + [(command >> i) & 1 for i in range(8)] + [1]:
                ctx.set(dut.rx, bit)
                for _ in range(DIVISOR):
                    await ctx.tick()
                    captured.append(ctx.get(dut.tx))

            ctx.set(dut.rx, 1)

            # Long enough for the longest reply (18 bytes) plus turnaround.
            for _ in range(DIVISOR * 10 * 22):
                await ctx.tick()
                captured.append(ctx.get(dut.tx))

        sim.add_testbench(drive)
        sim.run()
        return self._decode(captured)

    @staticmethod
    def _decode(samples):
        """Decode 8-N-1 bytes from a sampled line, as a UART receiver would."""
        decoded = []
        index = 0
        while index < len(samples) - DIVISOR * 10:
            if samples[index] == 1 and samples[index + 1] == 0:
                start = index + 1
                byte = 0
                for bit in range(8):
                    # Sample at bit centre.
                    position = start + DIVISOR // 2 + DIVISOR * (bit + 1)
                    byte |= samples[position] << bit
                decoded.append(byte)
                index = start + DIVISOR * 10
            else:
                index += 1
        return decoded

    def test_status_returns_status_and_crc(self):
        """The shortest response: status byte then CRC, nothing else."""
        response = self._run(CMD_STATUS)
        self.assertEqual(len(response), 2,
                         f"expected 2 bytes, got {[hex(b) for b in response]}")
        status, crc = response
        self.assertTrue(status & (1 << STATUS_OK), "OK bit not set")
        self.assertEqual(crc, crc8([status]), "CRC mismatch")

    def test_ping_reports_protocol_version(self):
        response = self._run(CMD_PING)
        self.assertEqual(len(response), 4,
                         f"expected 4 bytes, got {[hex(b) for b in response]}")
        status, version, build, crc = response
        self.assertTrue(status & (1 << STATUS_OK))
        self.assertEqual(version, PROTOCOL_VERSION)
        self.assertEqual(crc, crc8([status, version, build]), "CRC mismatch")

    def test_power_returns_sixteen_bytes(self):
        """POWER carries VBUS x4 and VSENSE x4 as little-endian 16-bit values."""
        # Distinguishable per byte, so a transposition or an off-by-one in the
        # word select shows up as a wrong value rather than a plausible one.
        values = [0x1122, 0x3344, 0x5566, 0x7788,
                  0x99AA, 0xBBCC, 0xDDEE, 0xF001]
        packed = 0
        for index, value in enumerate(values):
            packed |= value << (16 * index)

        response = self._run(CMD_POWER, power_data=packed)
        self.assertEqual(len(response), 18,
                         f"expected 18 bytes, got {len(response)}")

        status, payload, crc = response[0], response[1:17], response[17]
        self.assertTrue(status & (1 << STATUS_OK))

        expected = []
        for value in values:
            expected += [value & 0xFF, (value >> 8) & 0xFF]
        self.assertEqual(payload, expected,
                         "payload bytes do not match the input, little-endian")
        self.assertEqual(crc, crc8([status] + payload), "CRC mismatch")

    def test_unknown_command_clears_ok(self):
        """An unrecognised command must still answer, with OK clear.

        A well-formed rejection lets the master distinguish 'not understood'
        from 'not there' -- a timeout cannot.
        """
        # 0xF0 is outside every allocated range: not PING/STATUS/POWER,
        # and not in the 0x40-0x7F LED block.
        response = self._run(0xF0)
        self.assertEqual(len(response), 2,
                         f"expected 2 bytes, got {[hex(b) for b in response]}")
        status, crc = response
        self.assertFalse(status & (1 << STATUS_OK),
                         "OK bit set for an unknown command")
        self.assertEqual(crc, crc8([status]))

    def test_status_flags_are_reported(self):
        """The flag inputs must appear in the status byte."""
        response = self._run(CMD_STATUS, events=1, error=1)
        status = response[0]
        self.assertTrue(status & (1 << 1), "events bit not set")
        self.assertTrue(status & (1 << 2), "error bit not set")

    def test_heartbeat_toggles_between_responses(self):
        """The heartbeat bit must change, or it proves nothing.

        A static value could be a wedged state machine replaying a stale
        buffer; a value that changes proves the responder is executing.
        """
        # Two commands into a single instance: a fresh one restarts at
        # heartbeat=0, so comparing across instances would compare two zeros.
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)
        captured = []

        async def drive(ctx):
            # Capture continuously from the first cycle, as _run() does --
            # starting later lands mid-transmission and the decoder finds no
            # idle-to-start edge.
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 2):
                await ctx.tick()
                captured.append(ctx.get(dut.tx))
            for _ in range(2):
                for bit in ([0] + [(CMD_STATUS >> i) & 1 for i in range(8)] + [1]):
                    ctx.set(dut.rx, bit)
                    for _ in range(DIVISOR):
                        await ctx.tick()
                        captured.append(ctx.get(dut.tx))
                ctx.set(dut.rx, 1)
                for _ in range(DIVISOR * 10 * 8):
                    await ctx.tick()
                    captured.append(ctx.get(dut.tx))

        sim.add_testbench(drive)
        sim.run()

        responses = self._decode(captured)
        self.assertGreaterEqual(len(responses), 4,
                                "expected two 2-byte responses")
        first_hb = (responses[0] >> STATUS_HEARTBEAT) & 1
        second_hb = (responses[2] >> STATUS_HEARTBEAT) & 1
        self.assertNotEqual(first_hb, second_hb,
                            "heartbeat did not toggle between responses")

    def test_line_idles_high_when_not_responding(self):
        """The line must rest high, or there is no falling start bit to find."""
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)
        samples = []

        async def idle(ctx):
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 20):
                await ctx.tick()
                samples.append(ctx.get(dut.tx))

        sim.add_testbench(idle)
        sim.run()
        self.assertTrue(all(s == 1 for s in samples),
                        "line was driven low with no command pending")

    def test_leds_indicate_activity(self):
        """The status LEDs must reflect what the responder is doing.

        A link that fails silently is hard to diagnose on a bench; these make
        the responder's state visible without attaching a debugger.
        """
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)
        seen = []

        async def drive(ctx):
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 2):
                await ctx.tick()
                seen.append(ctx.get(dut.leds))
            for bit in [0] + [(CMD_POWER >> i) & 1 for i in range(8)] + [1]:
                ctx.set(dut.rx, bit)
                for _ in range(DIVISOR):
                    await ctx.tick()
                    seen.append(ctx.get(dut.leds))
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 10 * 22):
                await ctx.tick()
                seen.append(ctx.get(dut.leds))

        sim.add_testbench(drive)
        sim.run()

        self.assertTrue(any(v & 0b000001 for v in seen),
                        "no LED showed a command byte arriving")
        self.assertTrue(any(v & 0b000010 for v in seen),
                        "no LED showed a response being transmitted")
        self.assertTrue(any(v & 0b001000 for v in seen),
                        "the POWER-command LED never lit")
        self.assertFalse(any(v & 0b100000 for v in seen),
                         "the unknown-command LED lit for a valid command")

    def test_led_command_sets_the_pattern(self):
        """An LED command must latch its pattern and acknowledge.

        The pattern lives in the low six bits of the opcode, so the whole
        command fits in one byte and the protocol stays stateless.
        """
        for pattern in (0b000000, 0b101010, 0b111111):
            dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
            m = Module()
            m.submodules.dut = dut
            sim = Simulator(m)
            sim.add_clock(1 / CLK_HZ)
            captured = []
            seen = []

            async def drive(ctx, p=pattern):
                ctx.set(dut.rx, 1)
                for _ in range(DIVISOR * 2):
                    await ctx.tick()
                    captured.append(ctx.get(dut.tx))
                command = CMD_LED_BASE | p
                for bit in [0] + [(command >> i) & 1 for i in range(8)] + [1]:
                    ctx.set(dut.rx, bit)
                    for _ in range(DIVISOR):
                        await ctx.tick()
                        captured.append(ctx.get(dut.tx))
                ctx.set(dut.rx, 1)
                for _ in range(DIVISOR * 10 * 12):
                    await ctx.tick()
                    captured.append(ctx.get(dut.tx))
                    seen.append((ctx.get(dut.led_override),
                                 ctx.get(dut.led_pattern)))

            sim.add_testbench(drive)
            sim.run()

            response = self._decode(captured)
            self.assertEqual(len(response), 2,
                             f"pattern {pattern:06b}: expected a 2-byte ack")
            self.assertTrue(response[0] & (1 << STATUS_OK),
                            f"pattern {pattern:06b}: OK bit not set")
            self.assertTrue(any(o and v == pattern for o, v in seen),
                            f"pattern {pattern:06b} was never latched")

    def test_button_press_latches_until_reported(self):
        """A momentary press must survive until the master has seen it.

        Without a latch a press landing between two polls is lost entirely.
        The latch clears only after a response carrying it has been fully
        transmitted, so a press is not dropped when a reply goes missing.
        """
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)
        captured = []

        async def drive(ctx):
            ctx.set(dut.rx, 1)
            ctx.set(dut.button, 0)
            for _ in range(DIVISOR):
                await ctx.tick()
                captured.append(ctx.get(dut.tx))

            # A brief press, well before any command arrives.
            ctx.set(dut.button, 1)
            for _ in range(DIVISOR // 2):
                await ctx.tick()
                captured.append(ctx.get(dut.tx))
            ctx.set(dut.button, 0)
            for _ in range(DIVISOR):
                await ctx.tick()
                captured.append(ctx.get(dut.tx))

            # Two STATUS polls: the first should report it, the second not.
            for _ in range(2):
                for bit in [0] + [(CMD_STATUS >> i) & 1 for i in range(8)] + [1]:
                    ctx.set(dut.rx, bit)
                    for _ in range(DIVISOR):
                        await ctx.tick()
                        captured.append(ctx.get(dut.tx))
                ctx.set(dut.rx, 1)
                for _ in range(DIVISOR * 10 * 8):
                    await ctx.tick()
                    captured.append(ctx.get(dut.tx))

        sim.add_testbench(drive)
        sim.run()

        responses = self._decode(captured)
        self.assertGreaterEqual(len(responses), 4,
                                "expected two 2-byte responses")
        first, second = responses[0], responses[2]
        self.assertTrue(first & (1 << STATUS_EVENTS),
                        "the press was not reported in the first poll")
        self.assertFalse(second & (1 << STATUS_EVENTS),
                         "the latch did not clear after being reported")

    def test_tx_active_is_low_when_idle(self):
        """tx_active drives the tri-state; asserting it while idle would hold
        the shared wire and block Apollo from transmitting."""
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)
        samples = []

        async def idle(ctx):
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 20):
                await ctx.tick()
                samples.append(ctx.get(dut.tx_active))

        sim.add_testbench(idle)
        sim.run()
        self.assertTrue(all(s == 0 for s in samples),
                        "tx_active asserted while idle")


if __name__ == "__main__":
    unittest.main()
