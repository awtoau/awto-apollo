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
    SidebandResponder, CMD_PING, CMD_STATUS, CMD_POWER, CMD_DEVICES,
    CMD_LED_RELEASE,
    STATUS_OK, STATUS_HEARTBEAT, STATUS_EVENTS, PROTOCOL_VERSION,
    CMD_LED_BASE, PAYLOAD_SIZE, MAX_PAYLOAD, FIRMWARE_ADV_RESPONSE_MAX,
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

    def test_devices_returns_flash_id(self):
        """DEVICES carries the JEDEC ID and a presence-flags byte."""
        response = self._run(CMD_DEVICES,
                             flash_manufacturer=0xEF,   # Winbond
                             flash_memory_type=0x40,
                             flash_capacity=0x16,       # 4 MiB
                             flash_valid=1,
                             hyperram_present=1)
        self.assertEqual(len(response), 6,
                         f"expected 6 bytes, got {len(response)}")

        status, payload, crc = response[0], response[1:5], response[5]
        self.assertTrue(status & (1 << STATUS_OK))
        self.assertEqual(payload[:3], [0xEF, 0x40, 0x16],
                         "JEDEC ID bytes are out of order or wrong")
        self.assertEqual(payload[3], 0b11, "both presence flags should be set")
        self.assertEqual(crc, crc8([status] + payload), "CRC mismatch")

    def test_devices_clears_ok_before_flash_read(self):
        """Until the flash has answered, DEVICES must report OK clear.

        Otherwise the power-on zeros are indistinguishable from a device that
        genuinely identified itself as 00 00 00.
        """
        response = self._run(CMD_DEVICES, flash_valid=0)
        status = response[0]
        self.assertFalse(status & (1 << STATUS_OK),
                         "OK must be clear while the flash ID is unread")

    def test_led_override_can_be_released(self):
        """An LED override must be reversible.

        It latches on any opcode in 0x40-0x7F and nothing else clears it, so
        without a release the responder has a state it cannot leave. That range
        is a quarter of all byte values, which made it reachable by accident
        before framing errors were rejected.
        """
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
        m = Module()
        m.submodules.dut = dut

        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)

        result = {}

        async def bench(ctx):
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 2):
                await ctx.tick()

            # Set a pattern, then release it.
            for command in (CMD_LED_BASE | 0b010101, CMD_LED_RELEASE):
                for bit in ([0] + [(command >> i) & 1 for i in range(8)] + [1]):
                    ctx.set(dut.rx, bit)
                    for _ in range(DIVISOR):
                        await ctx.tick()
                # Let the response finish before sending the next command.
                for _ in range(DIVISOR * 30):
                    await ctx.tick()

                result.setdefault("states", []).append(
                    (ctx.get(dut.led_override), ctx.get(dut.led_pattern)))

        sim.add_testbench(bench)
        sim.run()

        after_set, after_release = result["states"]
        self.assertEqual(after_set, (1, 0b010101),
                         "LED command should latch an override")
        self.assertEqual(after_release, (0, 0),
                         "CMD_LED_RELEASE should clear it")

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


class LineOwnershipTest(unittest.TestCase):
    """ Who is allowed to drive FPGA_ADV, and when.

    FPGA_ADV is a single wire with a CMOS driver at each end. Driving it
    push-pull at both ends means any timing slip that enables both drivers at
    once puts a driven high against a driven low: a low-impedance path through
    two output stages, and a hardware-damage risk rather than a link-reliability
    one. These tests measure the shared wire with both drivers modelled, so the
    claim is reproduced rather than reasoned about.

    Every test here has a push-pull counterpart that fails the same assertion --
    the positive control. Without it, a test that passes proves only that the
    test cannot detect the fault.
    """

    def _overlap(self, *, open_drain, master_open_drain, overlap_bits):
        """Have the master retry on top of a reply; count shorted cycles.

        A short needs a source and a sink -- one end driving high into the other
        driving low. Two ends pulling low together is just a low, which is the
        entire argument for open-drain, so "both drivers enabled" is not the
        thing to count.
        """
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD,
                                open_drain=open_drain)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)

        counts = {"both": 0, "shorted": 0}

        async def drive(ctx):
            frame = [0] + [(CMD_STATUS >> i) & 1 for i in range(8)] + [1]

            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 2):
                await ctx.tick()

            # The command.
            for bit in frame:
                ctx.set(dut.rx, bit)
                for _ in range(DIVISOR):
                    await ctx.tick()
            ctx.set(dut.rx, 1)

            # The master gives up early and retries, landing on a reply that is
            # still going out. This is the #99 timeout-retry case seen from the
            # electrical side.
            for _ in range(overlap_bits * DIVISOR):
                await ctx.tick()

            for bit in frame:
                ctx.set(dut.rx, bit)
                for _ in range(DIVISOR):
                    await ctx.tick()
                    fpga_oe = ctx.get(dut.pad_oe)
                    # An open-drain master only enables its driver for a low bit.
                    master_oe = (bit == 0) if master_open_drain else True
                    if fpga_oe and master_oe:
                        counts["both"] += 1
                        if ctx.get(dut.pad_o) != bit:
                            counts["shorted"] += 1

        sim.add_testbench(drive)
        sim.run()
        return counts

    # Overlaps to try. 1 and 2 bit times land in the turnaround and the status
    # byte; 8 and 12 land well inside the reply, where the responder is driving
    # continuously and a retry has the most to collide with.
    OVERLAPS = (1, 2, 4, 8, 12)

    def test_open_drain_both_ends_cannot_short(self):
        """The fix: with both ends open-drain, no overlap can short.

        This is the state the design was meant to be in.
        """
        for overlap in self.OVERLAPS:
            counts = self._overlap(open_drain=True, master_open_drain=True,
                                   overlap_bits=overlap)
            self.assertGreater(counts["both"], 0,
                               f"overlap {overlap}: the drivers never both "
                               f"enabled, so this proves nothing -- the test "
                               f"is not exercising a collision at all")
            self.assertEqual(counts["shorted"], 0,
                             f"overlap {overlap}: {counts['shorted']} cycles "
                             f"of driven-high into driven-low with both ends "
                             f"open-drain, which should be impossible")

    def test_push_pull_both_ends_does_short(self):
        """Positive control for the test above.

        If this passes with zero shorts, the detector is broken and the
        open-drain result means nothing.
        """
        worst = 0
        for overlap in self.OVERLAPS:
            counts = self._overlap(open_drain=False, master_open_drain=False,
                                   overlap_bits=overlap)
            worst = max(worst, counts["shorted"])
        self.assertGreater(worst, 0,
                           "push-pull at both ends produced no shorted cycles; "
                           "the detector cannot see the fault it is meant to "
                           "prove is fixed")

    def test_fixing_one_end_only_is_not_enough(self):
        """Both ends must change, which is why the firmware changed too.

        An open-drain FPGA still shorts against a push-pull master, and a
        push-pull FPGA still shorts against an open-drain master. Recorded as a
        test so a future change that reverts one side alone fails here rather
        than on a bench.
        """
        for open_drain, master_open_drain, label in (
                (True, False, "FPGA open-drain, master push-pull"),
                (False, True, "FPGA push-pull, master open-drain")):
            worst = 0
            for overlap in self.OVERLAPS:
                counts = self._overlap(open_drain=open_drain,
                                       master_open_drain=master_open_drain,
                                       overlap_bits=overlap)
                worst = max(worst, counts["shorted"])
            self.assertGreater(
                worst, 0,
                f"{label}: expected shorted cycles, since only one end is "
                f"open-drain -- if this is now zero the model has changed and "
                f"the reasoning behind the firmware change needs revisiting")

    def test_open_drain_never_drives_high(self):
        """The defining property: the driver is enabled only to pull low.

        Checked over a full 18-byte reply, which contains high bits, low bits,
        start bits, stop bits and the idle lead-in -- so any state that drives
        high shows up.
        """
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD, open_drain=True)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)
        driven_high = []

        async def drive(ctx):
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 2):
                await ctx.tick()
            for bit in [0] + [(CMD_POWER >> i) & 1 for i in range(8)] + [1]:
                ctx.set(dut.rx, bit)
                for _ in range(DIVISOR):
                    await ctx.tick()
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 10 * 24):
                await ctx.tick()
                if ctx.get(dut.pad_oe) and ctx.get(dut.pad_o):
                    driven_high.append(1)

        sim.add_testbench(drive)
        sim.run()
        self.assertEqual(len(driven_high), 0,
                         f"the open-drain driver sourced current on "
                         f"{len(driven_high)} cycles; it must only ever sink")

    def test_push_pull_drives_high_before_the_start_bit(self):
        """Positive control, and the specific window that made #88 concrete.

        tx_active asserts a SETTLE plus a LOAD ahead of the first bit reaching
        the wire, so oe = tx_active with o = tx claims the line by driving it
        HIGH for that lead-in -- while the master's own driver may still be
        enabled. This records that the lead-in exists and is non-zero.
        """
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD, open_drain=False)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)
        trace = []

        async def drive(ctx):
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 2):
                await ctx.tick()
            for bit in [0] + [(CMD_STATUS >> i) & 1 for i in range(8)] + [1]:
                ctx.set(dut.rx, bit)
                for _ in range(DIVISOR):
                    await ctx.tick()
            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 10 * 6):
                await ctx.tick()
                trace.append((ctx.get(dut.pad_oe), ctx.get(dut.pad_o)))

        sim.add_testbench(drive)
        sim.run()

        first_enabled = next(i for i, (oe, _) in enumerate(trace) if oe)
        first_low = next(i for i, (oe, o) in enumerate(trace) if oe and not o)
        self.assertGreater(
            first_low - first_enabled, 0,
            "expected the push-pull driver to be enabled while still driving "
            "high, before the start bit -- if this is now zero the FSM's "
            "SETTLE/LOAD timing changed and the #88 rationale should be re-read")

    def test_idle_releases_the_line_entirely(self):
        """With no command pending, the FPGA must not drive at all.

        Weaker than idling high: a driver held enabled at high would stop Apollo
        pulling the line low, and no command would ever arrive.
        """
        for open_drain in (True, False):
            dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD,
                                    open_drain=open_drain)
            m = Module()
            m.submodules.dut = dut
            sim = Simulator(m)
            sim.add_clock(1 / CLK_HZ)
            enabled = []

            async def idle(ctx):
                ctx.set(dut.rx, 1)
                for _ in range(DIVISOR * 20):
                    await ctx.tick()
                    enabled.append(ctx.get(dut.pad_oe))

            sim.add_testbench(idle)
            sim.run()
            self.assertFalse(any(enabled),
                             f"open_drain={open_drain}: the driver was enabled "
                             f"with no command pending")

    def test_open_drain_reply_is_still_decodable(self):
        """Open-drain must not change what is on the wire, only how.

        A pull-up-and-release line reads identically to a driven one at the
        receiver: released is high. Reconstructing the wire from pad_o/pad_oe --
        low when driven low, high when released -- must decode to the same reply
        the push-pull build produces. Otherwise the fix for #88 breaks the link
        it protects.
        """
        def reply(open_drain):
            dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD,
                                    open_drain=open_drain)
            m = Module()
            m.submodules.dut = dut
            sim = Simulator(m)
            sim.add_clock(1 / CLK_HZ)
            wire = []

            async def drive(ctx):
                def sample():
                    # The wire: driven low pulls it low, anything else is the
                    # pull-ups holding it high.
                    if ctx.get(dut.pad_oe) and not ctx.get(dut.pad_o):
                        return 0
                    return 1

                ctx.set(dut.rx, 1)
                for _ in range(DIVISOR * 2):
                    await ctx.tick()
                    wire.append(sample())
                for bit in [0] + [(CMD_POWER >> i) & 1 for i in range(8)] + [1]:
                    ctx.set(dut.rx, bit)
                    for _ in range(DIVISOR):
                        await ctx.tick()
                        wire.append(sample())
                ctx.set(dut.rx, 1)
                for _ in range(DIVISOR * 10 * 24):
                    await ctx.tick()
                    wire.append(sample())

            sim.add_testbench(drive)
            sim.run()
            return SidebandTest._decode(wire)

        open_drain_reply = reply(True)
        push_pull_reply = reply(False)
        self.assertEqual(len(open_drain_reply), 18,
                         f"open-drain reply was {len(open_drain_reply)} bytes, "
                         f"expected 18: {[hex(b) for b in open_drain_reply]}")
        self.assertEqual(open_drain_reply, push_pull_reply,
                         "open-drain and push-pull put different bytes on the "
                         "wire; the drive style must not change the protocol")


class RetryCorruptionTest(unittest.TestCase):
    """ What the responder does with a command that arrives mid-response.

    The firmware's timeout-retry bug depends on this: if a retry sent during a
    reply were queued and answered, the master would get a second reply and the
    counts would work out. It is not -- the responder forces its receiver
    idle-high for the whole of its own transmission, so the retry is not
    rejected, it is never received at all. The master therefore re-arms its
    collector against bytes still in flight from the FIRST reply, and assembles a
    full-length response out of them.

    That is why the fix is to drain the receiver before re-arming, and not to add
    framing or a sequence number.
    """

    def _send_two(self, gap_bits):
        """Send a command, then another `gap_bits` bit times later.

        Returns the decoded replies. A single reply means the second command was
        swallowed; two means it was received.
        """
        dut = SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD)
        m = Module()
        m.submodules.dut = dut
        sim = Simulator(m)
        sim.add_clock(1 / CLK_HZ)
        wire = []
        strobes = []

        async def drive(ctx):
            def sample():
                if ctx.get(dut.pad_oe) and not ctx.get(dut.pad_o):
                    return 0
                return 1

            async def send(command):
                for bit in ([0] + [(command >> i) & 1 for i in range(8)] + [1]):
                    ctx.set(dut.rx, bit)
                    for _ in range(DIVISOR):
                        await ctx.tick()
                        wire.append(sample())
                        if ctx.get(dut.rx_strobe):
                            strobes.append(ctx.get(dut.rx_byte))
                ctx.set(dut.rx, 1)

            ctx.set(dut.rx, 1)
            for _ in range(DIVISOR * 2):
                await ctx.tick()
                wire.append(sample())

            # First command: POWER, the longest reply, so there is plenty of
            # response still in flight to collide with.
            await send(CMD_POWER)

            for _ in range(gap_bits * DIVISOR):
                await ctx.tick()
                wire.append(sample())
                if ctx.get(dut.rx_strobe):
                    strobes.append(ctx.get(dut.rx_byte))

            # The retry.
            await send(CMD_STATUS)

            # Long enough for both replies, had both been answered.
            for _ in range(DIVISOR * 10 * 44):
                await ctx.tick()
                wire.append(sample())
                if ctx.get(dut.rx_strobe):
                    strobes.append(ctx.get(dut.rx_byte))

        sim.add_testbench(drive)
        sim.run()
        return SidebandTest._decode(wire), strobes

    def test_a_retry_during_a_reply_is_never_received(self):
        """The retry is swallowed, not answered.

        4 bit times after the command is well inside the reply. If the responder
        saw it, there would be a second STATUS reply on the wire and a second
        strobe; there is neither.
        """
        replies, strobes = self._send_two(gap_bits=4)
        self.assertEqual(strobes, [CMD_POWER],
                         f"expected only the first command to be received, got "
                         f"{[hex(b) for b in strobes]} -- if CMD_STATUS appears "
                         f"here the responder is no longer deaf during its own "
                         f"transmission and the firmware's drain-before-arm "
                         f"reasoning needs revisiting")
        # One 18-byte reply, not two replies.
        self.assertEqual(len(replies), 18,
                         f"expected a single 18-byte reply, got {len(replies)} "
                         f"bytes: {[hex(b) for b in replies]}")

    def test_a_command_after_the_reply_is_received(self):
        """Control: once the line is quiet again, commands land normally.

        Without this the test above could pass simply because the harness never
        delivers a second command at all.
        """
        # 220 bit times is comfortably past an 18-byte reply plus turnaround.
        replies, strobes = self._send_two(gap_bits=220)
        self.assertEqual(strobes, [CMD_POWER, CMD_STATUS],
                         f"a command sent well after the reply should be "
                         f"received; got {[hex(b) for b in strobes]}")
        self.assertEqual(len(replies), 20,
                         f"expected an 18-byte and a 2-byte reply, got "
                         f"{len(replies)} bytes")


class PayloadWidthTest(unittest.TestCase):
    """ The gateware's payload width against the firmware's response buffer.

    These were two independent numbers -- payload_len as range(17) here and
    ADV_RESPONSE_MAX as 18 in the firmware -- with nothing tying them together
    and zero headroom. A 17-byte command would have truncated silently in
    gateware while firmware accepted the longer reply.
    """

    def test_every_payload_fits_the_firmware_buffer(self):
        """The assertion that now runs at elaboration, checked directly."""
        for command, size in PAYLOAD_SIZE.items():
            self.assertLessEqual(
                size, MAX_PAYLOAD,
                f"command {command:#04x} declares a {size}-byte payload, but "
                f"the firmware's {FIRMWARE_ADV_RESPONSE_MAX}-byte buffer can "
                f"only carry {MAX_PAYLOAD} (status + payload + CRC)")

    def test_the_assertion_actually_fires(self):
        """Positive control: an oversized payload must fail elaboration.

        Without this, the assertion could be unreachable and the test above
        would still pass.
        """
        from amaranth.hdl import Fragment
        import apollo_fpga.gateware.sideband as sideband

        original = dict(sideband.PAYLOAD_SIZE)
        try:
            # One byte too many for the firmware's buffer.
            sideband.PAYLOAD_SIZE[0xFE] = MAX_PAYLOAD + 1
            with self.assertRaises(AssertionError):
                Fragment.get(SidebandResponder(clk_freq_hz=CLK_HZ, baud=BAUD),
                             None)
        finally:
            sideband.PAYLOAD_SIZE.clear()
            sideband.PAYLOAD_SIZE.update(original)

    def test_payload_len_has_headroom(self):
        """payload_len must be able to hold MAX_PAYLOAD, not merely the largest
        payload in use -- otherwise the assertion guards a register that cannot
        represent the limit it checks against."""
        self.assertGreaterEqual(MAX_PAYLOAD, max(PAYLOAD_SIZE.values()))
        # And the widened register must still transmit the largest reply, so the
        # width change is not merely bigger but correct.
        response = SidebandTest("test_power_returns_sixteen_bytes")._run(
            CMD_POWER, power_data=0)
        self.assertEqual(len(response), 18,
                         f"largest reply came back as {len(response)} bytes "
                         f"after widening payload_len")


if __name__ == "__main__":
    unittest.main()
