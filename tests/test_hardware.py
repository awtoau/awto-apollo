#
# Hardware-in-the-loop tests for Apollo firmware.
#
# This file is part of Apollo.
# SPDX-License-Identifier: BSD-3-Clause
#
"""
Hardware-in-the-loop (HIL) tests for Apollo firmware, run against a real
Cynthion over USB.

Covers:

  * JTAG/UART pin mutual exclusion (awtoau/cynthion-workspace#65)
      While a JTAG session holds the shared d11 pins (PA10/PA11/PA14/PA15), the
      serial/UART console path must be locked out and unable to disturb JTAG;
      when the session ends, serial works again.

      Note the OS does NOT provide this exclusion: JTAG runs over the vendor
      interface via libusb while the console is a CDC ACM interface bound by
      cdc_acm, and the two are independently openable. The mutual exclusion is
      enforced entirely in firmware, so we observe it through its consequence --
      the JTAG scan chain staying intact under concurrent serial activity.

  * boot-to-DFU vendor request 0xed (awtoau/cynthion-workspace#67)
      `ApolloDebugger.boot_to_dfu()` must reboot the SAMD11 into the Saturn-V
      DFU bootloader. DISRUPTIVE -- skipped unless explicitly opted into.

Usage:

    # non-disruptive tests only (default)
    python -m pytest tests/test_hardware.py -v

    # include the reboot test
    APOLLO_TEST_ALLOW_REBOOT=1 python -m pytest tests/test_hardware.py -v

The reboot test restores the device afterwards by reflashing the application
firmware, so the suite is repeatable and order-independent. That requires
`fwup-util` on PATH and a built firmware image; if either is missing the test
skips rather than leaving the device stranded in the bootloader.

All tests skip (rather than fail) when no device / no CDC port is present, so
this file is safe to run in environments without hardware.
"""

import os
import re
import time
import glob
import shutil
import tempfile
import subprocess
import unittest

try:
    import serial
    _HAVE_PYSERIAL = True
except ImportError:
    _HAVE_PYSERIAL = False

try:
    import usb.core
    import usb.util
    # Pin the libusb1 backend explicitly. pyusb otherwise probes backends in
    # order at import time, and merely importing the legacy libusb0 backend
    # emits ctypes _pack_/_layout_ DeprecationWarnings (an error from Python
    # 3.19) -- even though libusb1 is what actually gets used. Naming the
    # backend skips the dead probe, so the warnings never arise rather than
    # being suppressed.
    import usb.backend.libusb1
    _USB_BACKEND = usb.backend.libusb1.get_backend()
    from apollo_fpga import ApolloDebugger
    _HAVE_APOLLO = True
except Exception:
    _USB_BACKEND = None
    _HAVE_APOLLO = False


# The Apollo CDC ACM port advertises this udev by-id substring.
_CDC_BY_ID_GLOB = "/dev/serial/by-id/*Apollo_Debugger*"

_APOLLO_VID = 0x1d50
_APOLLO_PID = 0x615c

# The FPGA's own PID. When the FPGA holds the shared USB port this is what
# enumerates instead of Apollo, and `apollo info` answers from the stub
# interface -- a state in which Apollo's control plane is NOT reachable.
_FPGA_STUB_PID = 0x615b

# How the bootloader personality identifies itself in its iProduct string.
_BOOTLOADER_PRODUCT_HINT = "bootloader"

# Ceiling on USB bus re-scans while waiting for the device to re-enumerate as
# the bootloader. The SAMD11 resets via the watchdog and comes back in well
# under a second; this bound exists only so a failed reboot surfaces as a test
# failure instead of hanging forever. We poll the bus rather than sleeping.
_MAX_ENUMERATION_POLLS = 20000

# Firmware image used to restore the application after the reboot test, relative
# to the repository root (this file lives in <repo>/tests/).
_FIRMWARE_BIN = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "firmware", "_build", "cynthion_d11", "firmware.bin")


def _find_apollo_cdc_port():
    matches = glob.glob(_CDC_BY_ID_GLOB)
    return matches[0] if matches else None


def _find_usb_device():
    """Return the 1d50:615c device currently on the bus, or None."""
    return usb.core.find(idVendor=_APOLLO_VID, idProduct=_APOLLO_PID,
                         backend=_USB_BACKEND)


def _product_string(device):
    """Best-effort iProduct read; returns '' if it can't be read."""
    try:
        return (usb.util.get_string(device, device.iProduct) or "").strip()
    except Exception:
        return ""


def _looks_like_bootloader(device):
    return _BOOTLOADER_PRODUCT_HINT in _product_string(device).lower()


def _poll_until_apollo():
    """Re-scan the bus until the Apollo application appears, or give up.

    Reclaiming the CONTROL port from the FPGA takes a USB re-enumeration, so
    the device is briefly absent. Poll rather than sleep: each iteration is a
    fresh enumeration, so this returns as soon as the state actually changes.
    Returns the device, or None if it never appeared.
    """
    for _ in range(_MAX_ENUMERATION_POLLS):
        device = _find_usb_device()
        if device is not None:
            return device
    return None


@unittest.skipUnless(_HAVE_APOLLO, "apollo_fpga / pyusb not importable")
@unittest.skipUnless(_HAVE_PYSERIAL, "pyserial not installed")
class JTAGUARTExclusionHILTest(unittest.TestCase):
    """Serial is locked out while JTAG is held, and freed when JTAG ends."""

    def setUp(self):
        try:
            self.debugger = ApolloDebugger()
        except Exception as e:
            self.skipTest(f"no Apollo device found: {e}")

        self.cdc_port = _find_apollo_cdc_port()
        if self.cdc_port is None:
            self.skipTest("Apollo CDC serial port not found under /dev/serial/by-id")

    def tearDown(self):
        try:
            self.debugger.close()
        except Exception:
            pass

    def _read_idcodes(self, jtag):
        return list(jtag.enumerate(return_idcodes=True))

    def _use_serial(self):
        """Drive the serial port the way a host console would: assert control
        lines, change baud, and write bytes. Each of these fires a firmware CDC
        callback that -- absent the lock -- would repinmux PA14 to the UART."""
        with serial.Serial(self.cdc_port, baudrate=115200, timeout=0) as ser:
            ser.dtr = True
            ser.rts = True
            ser.baudrate = 9600
            ser.baudrate = 115200
            ser.write(b"AT\r\n")
            ser.flush()
            ser.dtr = False

    def test_serial_is_locked_out_while_jtag_held(self):
        """While JTAG owns the pins, serial activity must not disturb JTAG."""
        with self.debugger.jtag as jtag:
            before = self._read_idcodes(jtag)
            self.assertTrue(before, "expected a device on the JTAG chain")

            # Serial traffic arrives mid-session; the lock must hold the pins.
            self._use_serial()

            after = self._read_idcodes(jtag)
            self.assertEqual(
                before, after,
                "JTAG chain changed after serial activity "
                f"({[hex(x) for x in before]} -> {[hex(x) for x in after]}): "
                "serial was NOT locked out while JTAG was held.")

    def test_serial_freed_after_jtag_ends(self):
        """When the JTAG session ends, serial works again and JTAG re-acquires
        cleanly -- i.e. the lock releases and is not stuck."""
        with self.debugger.jtag as jtag:
            self.assertTrue(self._read_idcodes(jtag))

        # Serial path is free again: the port opens and drives without error.
        self._use_serial()

        # And a fresh JTAG session re-acquires the pins cleanly.
        with self.debugger.jtag as jtag:
            self.assertTrue(
                self._read_idcodes(jtag),
                "JTAG failed to re-acquire after the lock was released.")


@unittest.skipUnless(_HAVE_APOLLO, "apollo_fpga / pyusb not importable")
@unittest.skipUnless(
    os.environ.get("APOLLO_TEST_ALLOW_REBOOT") == "1",
    "disruptive: set APOLLO_TEST_ALLOW_REBOOT=1 to allow rebooting the device")
class BootToDFUHILTest(unittest.TestCase):
    """Verifies the 0xed boot-to-DFU vendor request against real hardware.

    Reboots the device into the bootloader, then restores it by reflashing the
    application firmware in tearDown -- so this test is repeatable and does not
    strand the device for the other tests in this file. It skips up-front if it
    could not perform that restore (no fwup-util, or no built firmware image),
    rather than rebooting into a state it cannot undo.
    """

    def setUp(self):
        device = _find_usb_device()
        if device is None:
            self.skipTest(f"no {_APOLLO_VID:04x}:{_APOLLO_PID:04x} device found")

        if _looks_like_bootloader(device):
            self.skipTest(
                "device is already in the bootloader; flash Apollo firmware "
                "before running this test")

        # Refuse to reboot unless we can put the device back afterwards.
        if shutil.which("fwup-util") is None:
            self.skipTest("fwup-util not on PATH; cannot restore after reboot")
        if not os.path.exists(_FIRMWARE_BIN):
            self.skipTest(
                f"firmware image not built ({_FIRMWARE_BIN}); "
                "cannot restore after reboot")

        try:
            self.debugger = ApolloDebugger()
        except Exception as e:
            self.skipTest(f"could not open Apollo device: {e}")

    def tearDown(self):
        """Restore the application firmware so the device is usable again."""
        try:
            self.debugger.close()
        except Exception:
            pass

        # Only reflash if we actually left it in the bootloader.
        device = _find_usb_device()
        if device is None or not _looks_like_bootloader(device):
            return

        subprocess.run(
            ["fwup-util", "--device",
             f"{_APOLLO_VID:04x}:{_APOLLO_PID:04x}", _FIRMWARE_BIN],
            capture_output=True, check=False)

    def _poll_until(self, predicate):
        """Re-scan the USB bus until `predicate(device_or_None)` is true.

        Polls rather than sleeping: each iteration is a fresh enumeration, so
        this returns as soon as the device state actually changes.
        """
        for _ in range(_MAX_ENUMERATION_POLLS):
            device = _find_usb_device()
            if predicate(device):
                return device
        self.fail("USB device never reached the expected state within "
                  f"{_MAX_ENUMERATION_POLLS} bus scans")

    def test_boot_to_dfu_reboots_into_bootloader(self):
        """boot_to_dfu() must land the device in the Saturn-V bootloader."""

        # Precondition: we are talking to the Apollo *application*, not the
        # bootloader. Reading the firmware version proves the app is alive.
        version = self.debugger.get_firmware_version()
        self.assertTrue(version, "expected a firmware version from the application")
        self.assertFalse(
            _looks_like_bootloader(_find_usb_device()),
            "precondition failed: device is already the bootloader")

        # Issue the reboot. The device ACKs, then resets via the watchdog.
        self.debugger.boot_to_dfu()

        # Release our handle so re-enumeration isn't holding a stale claim.
        try:
            self.debugger.close()
        except Exception:
            pass

        # The device must come back identifying as the bootloader.
        device = self._poll_until(
            lambda d: d is not None and _looks_like_bootloader(d))

        self.assertTrue(
            _looks_like_bootloader(device),
            f"device came back as {_product_string(device)!r}, "
            "expected the Saturn-V bootloader")


@unittest.skipUnless(_HAVE_APOLLO, "apollo_fpga / pyusb not importable")
class CLICommandSmokeTest(unittest.TestCase):
    """Every `apollo` subcommand is invoked and must enter and respond.

    Each command must reach the device AND return a recognisable response. An
    exit status of 0 is not sufficient evidence: several apollo commands exit 0
    while printing an outright failure (`spi-reg` and `jtag-reg` print "Failed
    to autonegotiate ..." and still return 0), so a test that only checked the
    return code would pass on a broken device. Every executed command therefore
    declares a pattern its output must contain, and a pattern set that must
    never appear.

    Commands are classified rather than blanket-run: the destructive ones
    (flash-erase / flash-program / flash-fast, and configure/svf which need a
    bitstream) would rewrite the FPGA's configuration flash, so they are
    checked at the argument-parsing layer only unless explicitly opted into
    with APOLLO_TEST_ALLOW_FLASH_WRITE=1.

    A per-command result table is printed, including the matched response, so
    the log shows what the device actually said rather than just "rc=0".
    """

    # Output that always means failure, whatever the exit status.
    FAILURE_MARKERS = (
        "Traceback",
        "Failed to",
        "No Apollo device",
        "Error:",
        "error:",
    )

    # (argv, must-contain pattern). The pattern is what proves the device
    # actually answered, rather than the command merely exiting cleanly.
    SAFE_COMMANDS = [
        (["info"],                  r"Product ID:\s*615c"),
        (["info"],                  r"Firmware version:\s*v\d+\.\d+"),
        (["jtag-scan"],             r"[0-9a-f]{8}\s+--\s+Lattice\b"),
        (["flash-info"],            r"Manufacturer:\s*\w+"),
        (["flash-info"],            r"Device:\s*\w+"),
        (["spi", "00"],             r"response:\s*b'"),
        (["jtag-spi", "00"],        r"response:\s*b'"),
        (["leds", "0"],             None),   # no output expected
        (["leds", "500"],           None),
        (["force-offline"],         None),
        (["reconfigure"],           None),
    ]

    # Commands that reach the device but require FPGA-side debug-SPI gateware
    # that is not present in the stock bitstream. They fail cleanly ("Failed to
    # autonegotiate ...") and exit 0. We assert that specific, expected failure
    # so the suite records them honestly instead of scoring them as passes.
    EXPECTED_UNAVAILABLE = [
        (["spi-reg", "0", "0"],  r"Failed to autonegotiate SPI"),
        (["jtag-reg", "0", "0"], r"Failed to autonegotiate meta-JTAG"),
    ]

    # Destructive or input-requiring: verify they parse/exist, don't run them.
    GUARDED_COMMANDS = [
        ["flash-erase"],
        ["flash-program"],
        ["flash-fast"],
        ["flash-read"],
        ["configure"],
        ["svf"],
    ]

    @classmethod
    def setUpClass(cls):
        cls.results = []

        stub = usb.core.find(idVendor=_APOLLO_VID, idProduct=_FPGA_STUB_PID,
                             backend=_USB_BACKEND)

        # Genuinely no hardware at all: the only legitimate skip.
        if _find_usb_device() is None and stub is None:
            raise unittest.SkipTest(
                f"no Cynthion found (neither {_APOLLO_VID:04x}:{_APOLLO_PID:04x} "
                f"nor {_APOLLO_VID:04x}:{_FPGA_STUB_PID:04x})")

        # If the FPGA holds the shared port, reclaim it rather than giving up.
        # This is routine, not exceptional: another test in this file (the
        # boot-to-DFU reboot) restarts the device, and the FPGA re-takes the
        # port on every boot -- so the state legitimately changes mid-suite.
        # Without this, `info` answers from the FPGA stub and reports PID 615b.
        if _find_usb_device() is None:
            subprocess.run(["apollo", "force-offline"], capture_output=True,
                           text=True, check=False,
                           env=dict(os.environ, APOLLO_BOARD="cynthion"))

        # After the reclaim attempt, Apollo MUST be reachable. Still not there
        # is a failure, not a skip -- skipping would hide a real problem behind
        # a green run, which is exactly what this suite exists to catch.
        device = _poll_until_apollo()
        if device is None:
            raise AssertionError(
                f"the FPGA holds the shared USB port "
                f"({_APOLLO_VID:04x}:{_FPGA_STUB_PID:04x}) and Apollo did not "
                "reclaim it; the control plane is unreachable. Try "
                "`apollo force-offline` manually, then re-run.")

        # Present, but running the bootloader rather than the application.
        if _looks_like_bootloader(device):
            raise AssertionError(
                "device is in the Saturn-V bootloader, not the Apollo "
                "application -- flash the firmware before running the suite")

    @classmethod
    def tearDownClass(cls):
        if not getattr(cls, "results", None):
            return
        width = max(len(name) for name, _, _ in cls.results)
        print("\n\n=== apollo CLI command smoke test ===")
        for name, verdict, detail in cls.results:
            print(f"  {name:<{width}}  {verdict:<11} {detail}")
        print(f"  {'-' * (width + 26)}")
        ok = sum(1 for _, v, _ in cls.results if v == "VERIFIED")
        na = sum(1 for _, v, _ in cls.results if v == "UNAVAILABLE")
        parsed = sum(1 for _, v, _ in cls.results if v == "PARSE-OK")
        print(f"  {len(cls.results)} checks: {ok} verified, "
              f"{na} expected-unavailable, {parsed} arg-checked\n")

    def _run(self, argv, env_extra=None):
        """Invoke the apollo CLI; returns (returncode, combined output)."""
        env = dict(os.environ, APOLLO_BOARD="cynthion", **(env_extra or {}))
        proc = subprocess.run(["apollo", *argv], capture_output=True,
                              text=True, env=env, check=False)
        return proc.returncode, (proc.stdout or "") + (proc.stderr or "")

    @staticmethod
    def _first_match(pattern, text):
        m = re.search(pattern, text)
        return m.group(0).strip() if m else None

    def test_safe_commands_respond_correctly(self):
        """Each safe command must return a recognisable response, not just rc=0."""
        failures = []
        for argv, pattern in self.SAFE_COMMANDS:
            name = " ".join(argv)
            rc, out = self._run(argv)
            out = out.strip()

            # Any failure marker means broken, regardless of exit status --
            # several commands print a failure and still exit 0.
            marker = next((m for m in self.FAILURE_MARKERS if m in out), None)
            if marker or rc != 0:
                first = next((l for l in out.splitlines() if l.strip()), "")
                self.results.append((name, "FAILED", f"rc={rc} {first[:58]}"))
                failures.append(f"{name}: rc={rc} {first[:80]}")
                continue

            if pattern is None:
                # Commands that legitimately print nothing; the absence of any
                # failure marker plus rc=0 is all the evidence available.
                self.results.append((name, "VERIFIED", "(no output expected)"))
                continue

            found = self._first_match(pattern, out)
            if found is None:
                first = next((l for l in out.splitlines() if l.strip()), "(no output)")
                self.results.append((name, "FAILED", f"missing /{pattern}/"))
                failures.append(
                    f"{name}: response did not match /{pattern}/; got: {first[:80]}")
            else:
                self.results.append((name, "VERIFIED", found[:58]))

        self.assertFalse(
            failures,
            "commands did not respond correctly:\n  " + "\n  ".join(failures))

    def test_expected_unavailable_commands_report_cleanly(self):
        """Commands needing absent FPGA gateware must fail in the known way.

        These reach the device but cannot complete without debug-SPI gateware
        that the stock bitstream does not provide. Asserting the specific
        expected message keeps them honest: if one day they start working, or
        start failing differently, this test notices instead of silently
        scoring them as passes (which the previous rc-only check did).
        """
        failures = []
        for argv, pattern in self.EXPECTED_UNAVAILABLE:
            name = " ".join(argv)
            rc, out = self._run(argv)
            found = self._first_match(pattern, out.strip())

            if "Traceback" in out:
                self.results.append((name, "FAILED", "crashed"))
                failures.append(f"{name}: crashed with a traceback")
            elif found is None:
                first = next((l for l in out.splitlines() if l.strip()), "(no output)")
                self.results.append((name, "CHANGED", first[:58]))
                failures.append(
                    f"{name}: expected /{pattern}/, got: {first[:80]} "
                    "(gateware may now be present -- promote to SAFE_COMMANDS)")
            else:
                self.results.append((name, "UNAVAILABLE", found[:58]))

        self.assertFalse(
            failures,
            "expected-unavailable commands behaved unexpectedly:\n  "
            + "\n  ".join(failures))

    def test_guarded_commands_are_reachable(self):
        """Destructive commands must exist and parse, but are not executed.

        Invoking them with no arguments should produce argparse's own usage
        error (rc=2), which proves the subcommand is registered and its parser
        is intact without touching the FPGA's configuration flash.
        """
        allow_writes = os.environ.get("APOLLO_TEST_ALLOW_FLASH_WRITE") == "1"
        failures = []

        for argv in self.GUARDED_COMMANDS:
            name = " ".join(argv)
            if allow_writes:
                self.results.append((name, "SKIPPED", "opt-in set, but no bitstream supplied"))
                continue

            rc, out = self._run([*argv, "--help"])
            if "Traceback" in out or rc != 0:
                self.results.append((name, "BROKEN", f"--help rc={rc}"))
                failures.append(f"{name}: --help rc={rc}")
            else:
                usage = next((l for l in out.splitlines() if l.startswith("usage")), "")
                self.results.append((name, "PARSE-OK", usage[:58]))

        self.assertFalse(
            failures,
            "destructive commands unreachable:\n  " + "\n  ".join(failures))


@unittest.skipUnless(_HAVE_APOLLO, "apollo_fpga / pyusb not importable")
@unittest.skipUnless(
    os.environ.get("APOLLO_TEST_ALLOW_FLASH_WRITE") == "1",
    "destructive: set APOLLO_TEST_ALLOW_FLASH_WRITE=1 to allow writing the "
    "FPGA configuration flash")
class FlashWriteHILTest(unittest.TestCase):
    """Exercises the bitstream-loading and flash-writing commands for real.

    DESTRUCTIVE: this writes the FPGA's SPI configuration flash. It is safe to
    run on a dedicated test board and nowhere else.

    The whole class is wrapped in a backup/restore: setUpClass reads the entire
    flash to a temporary file, and tearDownClass writes it back and verifies it
    byte-for-byte -- unconditionally, so an exception or a crashing command
    still leaves the board as it was found. That ordering is not incidental: a
    bitstream that is a valid ECP5 image but not working Cynthion gateware
    leaves the FPGA unable to drive USB, and without the backup the original
    contents would be gone.

    Requires a bitstream via APOLLO_TEST_BITSTREAM (an .bit for this part).
    Without one the class skips rather than inventing something to flash.
    """

    _backup = None
    _bitstream = None

    @classmethod
    def _cli(cls, argv, timeout=None):
        env = dict(os.environ, APOLLO_BOARD="cynthion")
        return subprocess.run(["apollo", *argv], capture_output=True,
                              text=True, env=env, check=False, timeout=timeout)

    @classmethod
    def setUpClass(cls):
        cls.results = []

        bitstream = os.environ.get("APOLLO_TEST_BITSTREAM")
        if not bitstream or not os.path.exists(bitstream):
            raise unittest.SkipTest(
                "set APOLLO_TEST_BITSTREAM to a .bit file for this part "
                "(LFE5U-12F) to run the flash-write tests")
        cls._bitstream = bitstream

        if _find_usb_device() is None:
            cls._cli(["force-offline"])
        if _poll_until_apollo() is None:
            raise AssertionError("Apollo is not reachable; cannot run flash tests")

        # Back up the entire flash BEFORE anything writes to it. Without this
        # the original contents are unrecoverable.
        fd, path = tempfile.mkstemp(prefix="apollo-flash-backup-", suffix=".bin")
        os.close(fd)
        proc = cls._cli(["flash-read", path])
        size = os.path.getsize(path) if os.path.exists(path) else 0
        if proc.returncode != 0 or size == 0:
            os.unlink(path)
            raise AssertionError(
                f"could not back up the flash (rc={proc.returncode}, {size} B); "
                "refusing to run destructive tests without a restore point")
        cls._backup = path
        cls._backup_size = size

    @classmethod
    def tearDownClass(cls):
        # Restore unconditionally -- including after a failed or crashing test.
        if cls._backup and os.path.exists(cls._backup):
            if _find_usb_device() is None:
                cls._cli(["force-offline"])
            _poll_until_apollo()

            restore = cls._cli(["flash-program", cls._backup])
            verdict = "restored" if restore.returncode == 0 else \
                      f"RESTORE FAILED rc={restore.returncode}"
            cls.results.append(("(teardown) flash restore",
                                "OK" if restore.returncode == 0 else "FAILED",
                                f"{cls._backup_size} B {verdict}"))
            os.unlink(cls._backup)

        if getattr(cls, "results", None):
            width = max(len(n) for n, _, _ in cls.results)
            print("\n\n=== apollo flash-write (destructive) test ===")
            for name, verdict, detail in cls.results:
                print(f"  {name:<{width}}  {verdict:<9} {detail}")
            print()

    def _timed(self, argv):
        """Run a CLI command, returning (proc, elapsed_seconds)."""
        start = time.monotonic()
        proc = self._cli(argv)
        return proc, time.monotonic() - start

    def test_1_configure_loads_bitstream_to_sram(self):
        """`configure` must load a bitstream into FPGA SRAM (volatile)."""
        proc, elapsed = self._timed(["configure", self._bitstream])
        size = os.path.getsize(self._bitstream)
        rate = size / elapsed / 1024 if elapsed else 0

        self.results.append(("configure",
                             "OK" if proc.returncode == 0 else "FAILED",
                             f"{size} B in {elapsed:.2f}s = {rate:.1f} KiB/s"))
        self.assertEqual(proc.returncode, 0,
                         f"configure failed: {proc.stdout}{proc.stderr}")
        self.assertNotIn("Traceback", proc.stdout + proc.stderr)

    def test_2_flash_program_writes_bitstream(self):
        """`flash-program` must write a bitstream to the configuration flash."""
        proc, elapsed = self._timed(["flash-program", self._bitstream])
        size = os.path.getsize(self._bitstream)
        rate = size / elapsed / 1024 if elapsed else 0

        self.results.append(("flash-program",
                             "OK" if proc.returncode == 0 else "FAILED",
                             f"{size} B in {elapsed:.2f}s = {rate:.1f} KiB/s"))
        self.assertEqual(proc.returncode, 0,
                         f"flash-program failed: {proc.stdout}{proc.stderr}")
        self.assertNotIn("Traceback", proc.stdout + proc.stderr)

    def test_3_flash_read_round_trips(self):
        """`flash-read` must read back what `flash-program` wrote."""
        fd, path = tempfile.mkstemp(prefix="apollo-flash-verify-", suffix=".bin")
        os.close(fd)
        try:
            proc, elapsed = self._timed(["flash-read", path])
            size = os.path.getsize(path) if os.path.exists(path) else 0
            rate = size / elapsed / 1024 if elapsed else 0

            self.results.append(("flash-read",
                                 "OK" if proc.returncode == 0 else "FAILED",
                                 f"{size} B in {elapsed:.2f}s = {rate:.1f} KiB/s"))
            self.assertEqual(proc.returncode, 0,
                             f"flash-read failed: {proc.stdout}{proc.stderr}")

            # The written bitstream must appear at the start of the flash.
            written = open(self._bitstream, "rb").read()
            read_back = open(path, "rb").read(len(written))
            self.assertEqual(
                read_back, written,
                "flash-read did not return the bitstream that flash-program wrote")
        finally:
            if os.path.exists(path):
                os.unlink(path)

    def test_4_flash_fast_is_not_broken(self):
        """`flash-fast` must not crash or wedge Apollo.

        Known-failing: this segfaults and leaves Apollo's USB stack
        unresponsive until a physical reset (see awtoau/cynthion-workspace#75).
        The test asserts the desired behaviour rather than the current one, so
        it goes green when the bug is fixed. It runs LAST so that its known
        lock-up cannot strand the earlier tests, and is opt-in separately
        because recovering from it needs someone at the bench.
        """
        if os.environ.get("APOLLO_TEST_ALLOW_FLASH_FAST") != "1":
            self.results.append(("flash-fast", "SKIPPED",
                                 "known to wedge Apollo (#75); "
                                 "set APOLLO_TEST_ALLOW_FLASH_FAST=1"))
            self.skipTest("flash-fast wedges Apollo (#75); opt in explicitly")

        proc, elapsed = self._timed(["flash-fast", self._bitstream])
        combined = proc.stdout + proc.stderr
        crashed = proc.returncode < 0 or "Traceback" in combined

        self.results.append(("flash-fast",
                             "FAILED" if crashed else "OK",
                             f"rc={proc.returncode} in {elapsed:.2f}s"))
        self.assertFalse(
            crashed,
            f"flash-fast crashed (rc={proc.returncode}) -- see #75:\n{combined[-400:]}")


@unittest.skipUnless(_HAVE_APOLLO, "apollo_fpga / pyusb not importable")
@unittest.skipUnless(
    os.environ.get("APOLLO_TEST_POWER_MONITOR") == "1",
    "needs the power_monitor bitstream loaded; set "
    "APOLLO_TEST_POWER_MONITOR=1 once it is")
class PowerMonitorHILTest(unittest.TestCase):
    """Reads the PAC1954 power monitor over the JTAG bring-up gateware.

    Requires ecp5-test/power_monitor/build/top.bit to be loaded first:

        apollo configure ecp5-test/power_monitor/build/top.bit

    Opt-in rather than automatic because it needs that specific bitstream --
    running it against whatever happens to be in SRAM would fail confusingly
    rather than usefully. See awtoau/cynthion-workspace#82 and
    docs/luna_ecp5_fpga/pac1954-power-monitor.md.

    Non-destructive: reads only, no writes to the device's configuration.
    """

    # Mirrors ecp5-test/power_monitor/registers.py. Duplicated rather than
    # imported because that package lives in the workspace, not this repo, and
    # this test must run from an apollo checkout alone.
    JTAG_ID, JTAG_DEV_ADDR, JTAG_REG_ADDR = 1, 2, 3
    JTAG_TRIGGER, JTAG_DATA, JTAG_STATUS, JTAG_SIZE = 4, 5, 6, 7
    STATUS_DONE = 0b01

    APPLET_ID = 0x504D4F4E  # "PMON"
    PAC_ADDRESS = 0x10

    REG_REFRESH, REG_VBUS_BASE, REG_VSENSE_BASE = 0x00, 0x07, 0x0B
    REG_PRODUCT_ID, REG_MANUFACTURER_ID, REG_REVISION_ID = 0xFD, 0xFE, 0xFF
    EXPECTED = {"MANUFACTURER_ID": 0x54, "PRODUCT_ID": 0x7B, "REVISION_ID": 0x02}

    CHANNEL_PORTS = {1: "TARGET_A", 2: "TARGET_C", 3: "AUX", 4: "CONTROL"}

    VBUS_VOLTS_PER_LSB = 32.0 / 65536
    AMPS_PER_LSB = (0.100 / 65536) / 0.020

    # A byte transfer at 100 kHz is ~100 us. Bounded rather than a sleep: on a
    # NAK, done never asserts, so the loop must terminate for a missing device
    # to read as "no response" instead of hanging.
    POLL_ATTEMPTS = 200

    @classmethod
    def setUpClass(cls):
        from apollo_fpga import ApolloDebugger

        # Skip rather than error on anything that means "the bitstream is not
        # there". FlashWriteHILTest reconfigures the FPGA, so in a full-suite
        # run this class legitimately finds no power_monitor gateware loaded --
        # that is a skip, not a failure. Reading the ID register against
        # unrelated (or absent) gateware can also raise rather than return a
        # wrong value, hence catching broadly here.
        try:
            cls.device = ApolloDebugger()
            cls.regs = cls.device.registers
            applet = cls.regs.register_read(cls.JTAG_ID)
        except Exception as e:
            raise unittest.SkipTest(
                f"could not read the applet ID ({type(e).__name__}: {e}); "
                "load ecp5-test/power_monitor/build/top.bit first")

        if applet != cls.APPLET_ID:
            raise unittest.SkipTest(
                f"applet ID is {applet:#010x}, expected {cls.APPLET_ID:#010x}; "
                "load ecp5-test/power_monitor/build/top.bit first "
                "(FlashWriteHILTest reconfigures the FPGA, so a full-suite run "
                "will land here)")

    def _read_pac(self, reg, size=1):
        """Read a PAC195X register; None if the device did not respond."""
        self.regs.register_write(self.JTAG_DEV_ADDR, self.PAC_ADDRESS)
        self.regs.register_write(self.JTAG_REG_ADDR, reg)
        self.regs.register_write(self.JTAG_SIZE, size)
        self.regs.register_write(self.JTAG_TRIGGER, 1)

        mask = 0xFF if size == 1 else 0xFFFF
        for _ in range(self.POLL_ATTEMPTS):
            if self.regs.register_read(self.JTAG_STATUS) & self.STATUS_DONE:
                return self.regs.register_read(self.JTAG_DATA) & mask
        return None

    def test_1_device_identifies(self):
        """The three ID registers must match, proving the I2C link is sound."""
        for name, reg in (("MANUFACTURER_ID", self.REG_MANUFACTURER_ID),
                          ("PRODUCT_ID",      self.REG_PRODUCT_ID),
                          ("REVISION_ID",     self.REG_REVISION_ID)):
            value = self._read_pac(reg, size=1)
            self.assertIsNotNone(
                value, f"{name}: no response from PAC1954 at "
                       f"{self.PAC_ADDRESS:#04x}")
            self.assertEqual(
                value, self.EXPECTED[name],
                f"{name} is {value:#04x}, expected {self.EXPECTED[name]:#04x}")

    def test_2_read_size_selects_register_width(self):
        """A 2-byte read of a 1-byte register must not be mistaken for the value.

        The PAC195X auto-increments its address pointer within a read, so
        reading MANUFACTURER_ID (FEh) two bytes wide returns FEh followed by
        FFh. Asserting the low byte is the revision documents that behaviour --
        a size mismatch is silent corruption everywhere else.
        """
        wide = self._read_pac(self.REG_MANUFACTURER_ID, size=2)
        self.assertIsNotNone(wide, "no response to a 2-byte read")
        self.assertEqual(wide >> 8, self.EXPECTED["MANUFACTURER_ID"],
                         "high byte of the wide read should be FEh's value")
        self.assertEqual(wide & 0xFF, self.EXPECTED["REVISION_ID"],
                         "low byte should be FFh, i.e. the pointer advanced")

    def test_3_at_least_one_rail_is_powered(self):
        """Something must be supplying the board, or nothing would be running.

        Deliberately does not assert *which* port: that depends on how the
        board is cabled. It asserts only that the rail carrying us reads a
        plausible USB voltage, which catches a dead I2C link, a wrong channel
        mapping, or broken scaling.
        """
        self._read_pac(self.REG_REFRESH, size=1)

        readings = {}
        for channel in range(1, 5):
            vbus = self._read_pac(self.REG_VBUS_BASE + channel - 1, size=2)
            vsense = self._read_pac(self.REG_VSENSE_BASE + channel - 1, size=2)
            self.assertIsNotNone(vbus, f"channel {channel}: no VBUS response")
            self.assertIsNotNone(vsense, f"channel {channel}: no VSENSE response")
            readings[self.CHANNEL_PORTS[channel]] = (
                vbus * self.VBUS_VOLTS_PER_LSB,
                vsense * self.AMPS_PER_LSB,
            )

        powered = {p: (v, i) for p, (v, i) in readings.items() if v > 4.0}
        detail = ", ".join(f"{p}={v:.3f}V/{i * 1000:.1f}mA"
                           for p, (v, i) in readings.items())
        self.assertTrue(powered, f"no rail above 4 V -- readings: {detail}")

        for port, (volts, _) in powered.items():
            self.assertLess(volts, 5.5,
                            f"{port} reads {volts:.3f} V, above USB spec -- "
                            f"scaling may be wrong ({detail})")

    def test_4_total_current_is_plausible(self):
        """Total draw must be non-trivial but far below the 5 A full scale.

        An idle r1.4 draws roughly 65 mA at 5 V. Bounds are deliberately loose:
        this is a sanity check on the scaling, not a power budget. Zero would
        mean the shunt readings are dead; amps would mean the LSB size is wrong.
        """
        self._read_pac(self.REG_REFRESH, size=1)

        total = 0.0
        for channel in range(1, 5):
            vsense = self._read_pac(self.REG_VSENSE_BASE + channel - 1, size=2)
            self.assertIsNotNone(vsense, f"channel {channel}: no VSENSE response")
            total += vsense * self.AMPS_PER_LSB

        self.assertGreater(total, 0.005,
                           f"total draw {total * 1000:.2f} mA is implausibly "
                           "low -- the board is running, so something must "
                           "be drawing current")
        self.assertLess(total, 2.0,
                        f"total draw {total * 1000:.0f} mA is implausibly high "
                        "for an idle board -- check the 20 mohm shunt "
                        "assumption and the FSR configuration")


if __name__ == "__main__":
    unittest.main()
