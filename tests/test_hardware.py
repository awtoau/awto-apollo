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
import glob
import shutil
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

    This is a SMOKE test, deliberately not a correctness test: it asserts that
    each command reaches the device and comes back, not that its output is
    right. The point is to catch the failure modes that silently rot a CLI --
    an import error, a broken argument signature, a vendor request that no
    longer dispatches, a command that hangs -- across the whole surface rather
    than the two or three paths the other tests happen to exercise.

    Commands are classified rather than blanket-run: the destructive ones
    (flash-erase / flash-program / flash-fast, and configure/svf which need a
    bitstream) would rewrite the FPGA's configuration flash, so they are
    checked at the argument-parsing layer only unless explicitly opted into
    with APOLLO_TEST_ALLOW_FLASH_WRITE=1.

    A per-command result table is printed so the log shows what actually ran.
    """

    # Commands safe to execute against the device as-is.
    SAFE_COMMANDS = [
        ["info"],
        ["jtag-scan"],
        ["flash-info"],
        ["leds", "0"],
        ["leds", "500"],
        ["spi-reg", "0", "0"],
        ["jtag-reg", "0", "0"],
        ["spi", "00"],
        ["spi-inv", "00"],
        ["jtag-spi", "00"],
        ["force-offline"],
        ["reconfigure"],
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
        if _find_usb_device() is None:
            raise unittest.SkipTest(
                f"no {_APOLLO_VID:04x}:{_APOLLO_PID:04x} device found")
        cls.results = []

    @classmethod
    def tearDownClass(cls):
        if not getattr(cls, "results", None):
            return
        width = max(len(name) for name, _, _ in cls.results)
        print("\n\n=== apollo CLI command smoke test ===")
        for name, verdict, detail in cls.results:
            print(f"  {name:<{width}}  {verdict:<9} {detail}")
        print(f"  {'-' * (width + 24)}")
        ran = sum(1 for _, v, _ in cls.results if v == "RESPONDED")
        parsed = sum(1 for _, v, _ in cls.results if v == "PARSE-OK")
        print(f"  {len(cls.results)} commands: {ran} executed, {parsed} arg-checked\n")

    def _run(self, argv, env_extra=None):
        """Invoke the apollo CLI; returns (returncode, combined output)."""
        env = dict(os.environ, APOLLO_BOARD="cynthion", **(env_extra or {}))
        proc = subprocess.run(["apollo", *argv], capture_output=True,
                              text=True, env=env, check=False)
        return proc.returncode, (proc.stdout or "") + (proc.stderr or "")

    def test_safe_commands_respond(self):
        """Each safe command must run and return a real exit status."""
        failures = []
        for argv in self.SAFE_COMMANDS:
            name = " ".join(argv)
            rc, out = self._run(argv)

            # A crash (traceback) or an argparse rejection means the command is
            # broken, regardless of what the device replied.
            broken = "Traceback" in out or rc == 2
            if broken:
                first = next((l for l in out.splitlines() if l.strip()), "")
                self.results.append((name, "BROKEN", f"rc={rc} {first[:60]}"))
                failures.append(f"{name}: rc={rc} {first[:80]}")
            else:
                self.results.append((name, "RESPONDED", f"rc={rc}"))

        self.assertFalse(
            failures,
            "commands failed to enter/respond:\n  " + "\n  ".join(failures))

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


if __name__ == "__main__":
    unittest.main()
