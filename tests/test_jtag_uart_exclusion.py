#
# Hardware-in-the-loop test for the JTAG/UART pin mutual-exclusion lock.
#
# This file is part of Apollo.
# SPDX-License-Identifier: BSD-3-Clause
#
"""
Hardware-in-the-loop (HIL) validation of the sticky JTAG/UART mode-exclusion
lock (apollo_mode.c / firmware guards; see awtoau/cynthion-workspace#65).

The contract under test, stated plainly:

    While a JTAG session holds the shared pins, the serial (UART console) path
    is locked out and cannot disturb JTAG. The lock is released -- and serial
    works again -- when the JTAG session ends (or the device is reset).

On the Cynthion d11 board the console UART and JTAG share the same physical pins
(PA10/PA11/PA14/PA15) with no hardware arbitration; the IDCODE read over JTAG
depends on PA14 (=TDI) staying pinmuxed to JTAG. If the serial path were able to
re-init the UART mid-session it would repinmux PA14 and the chain read would
corrupt. So "serial is locked out" is observed as "JTAG stays intact"; "serial is
freed" is observed as "the port reopens and JTAG cleanly re-acquires".

Note: opening the CDC port from the host ALWAYS succeeds -- the endpoint is always
enumerated. The lock lives in firmware and only prevents the *pinmux* from being
stolen, so the host observes the lock through its consequence on JTAG, not by the
port refusing to open.

Requirements: a Cynthion running lock-bearing Apollo firmware, connected over USB.
Skips (does not fail) if no device / no CDC port is present.
"""

import glob
import unittest

try:
    import serial
    _HAVE_PYSERIAL = True
except ImportError:
    _HAVE_PYSERIAL = False

try:
    from apollo_fpga import ApolloDebugger
    _HAVE_APOLLO = True
except Exception:
    _HAVE_APOLLO = False


# The Apollo CDC ACM port advertises this udev by-id substring.
_CDC_BY_ID_GLOB = "/dev/serial/by-id/*Apollo_Debugger*"


def _find_apollo_cdc_port():
    matches = glob.glob(_CDC_BY_ID_GLOB)
    return matches[0] if matches else None


@unittest.skipUnless(_HAVE_APOLLO, "apollo_fpga not importable")
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

    # -- the contract, in two tests -------------------------------------------

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
        # Hold, then end the JTAG session (release path #1: explicit JTAG exit).
        with self.debugger.jtag as jtag:
            self.assertTrue(self._read_idcodes(jtag))

        # Serial path is free again: the port opens and drives without error.
        self._use_serial()

        # And a fresh JTAG session re-acquires the pins cleanly.
        with self.debugger.jtag as jtag:
            self.assertTrue(
                self._read_idcodes(jtag),
                "JTAG failed to re-acquire after the lock was released.")


if __name__ == "__main__":
    unittest.main()
