#
# This file is part of Apollo.
#
# Copyright (c) 2023 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" Controllers for communicating with Apollo through the FPGA_ADV pin """

from amaranth                       import Cat, Elaboratable, Module, Signal, Mux

from luna.gateware.usb.usb2.request import USBRequestHandler
from usb_protocol.types             import USBRequestType, USBRequestRecipient


class ApolloAdvertiser(Elaboratable):
    """ Gateware that implements an announcement signal for Apollo using the FPGA_ADV pin.

    Used to tell Apollo that the gateware wants to use the CONTROL port.
    Apollo will keep the port switch connected to the FPGA after a reset as long as this 
    signal is being received and the port takeover is allowed.

    I/O ports:
        I: stop -- Advertisement signal is stopped if this line is asserted.
    """
    def __init__(self, pad=None, clk_freq_hz=None):
        self.pad         = pad
        self.clk_freq_hz = clk_freq_hz
        self.stop        = Signal()

    def default_request_handler(self, if_number):
        return ApolloAdvertiserRequestHandler(if_number, self.stop)

    def elaborate(self, platform):
        m = Module()

        # Handle default values.
        if self.pad is None:
            self.pad = platform.request("int")
        if self.clk_freq_hz is None:
            self.clk_freq_hz = platform.DEFAULT_CLOCK_FREQUENCIES_MHZ["sync"] * 1e6

        # Generate clock with 20ms period.
        half_period = int(self.clk_freq_hz * 10e-3)
        timer       = Signal(range(half_period))
        clk         = Signal()
        m.d.sync   += timer.eq(Mux(timer == half_period-1, 0, timer+1))
        with m.If((timer == 0) & (~self.stop)):
            m.d.sync += clk.eq(~clk)

        # Drive the FPGA_ADV pin with the generated clock signal.
        m.d.comb += self.pad.o.eq(clk)
        
        return m


class ApolloUARTAdvertiser(Elaboratable):
    """ Advertises to Apollo over FPGA_ADV as a UART heartbeat rather than a pulse train.

    Transmits the framing pattern {0xC1, 0x14, 0x01, 0xA5} at 1 Mbaud, 8-N-1,
    once per interval. Apollo treats "a valid frame arrived within the last
    300 ms" as "the FPGA wants the port".

    This is the transmit half of the selectable advertisement mechanism:
    Apollo defaults to counting edges (ApolloAdvertiser above) and only listens
    for these frames once a host has switched it to UART mode. The two cannot
    run at once -- FPGA_ADV muxes to either EIC or SERCOM1 on the Apollo side --
    so a design instantiates one or the other, not both.

    Why a frame rather than more pulses: an edge counter cannot distinguish an
    advertising FPGA from a stuck or floating pin, whereas a 4-byte pattern at a
    known baud is not something noise produces. That is the point of the
    exercise -- see awtoau/cynthion-workspace#68.

    I/O ports:
        I: stop -- Advertisement is stopped if this line is asserted.
    """

    # Must match HEARTBEAT_PATTERN in Apollo's fpga_adv.c.
    PATTERN = [0xC1, 0x14, 0x01, 0xA5]

    def __init__(self, pad=None, clk_freq_hz=None, baud=1_000_000,
                 interval_ms=100):
        self.pad         = pad
        self.clk_freq_hz = clk_freq_hz
        self.baud        = baud
        # Well inside Apollo's 300 ms timeout, so a single lost frame does not
        # surrender the port.
        self.interval_ms = interval_ms
        self.stop        = Signal()

    def default_request_handler(self, if_number):
        return ApolloAdvertiserRequestHandler(if_number, self.stop)

    def elaborate(self, platform):
        m = Module()

        if self.pad is None:
            self.pad = platform.request("int")
        if self.clk_freq_hz is None:
            self.clk_freq_hz = platform.DEFAULT_CLOCK_FREQUENCIES_MHZ["sync"] * 1e6

        divisor    = int(self.clk_freq_hz // self.baud)
        bit_timer  = Signal(range(divisor))
        bit_strobe = Signal()

        m.d.comb += bit_strobe.eq(bit_timer == 0)
        m.d.sync += bit_timer.eq(Mux(bit_strobe, divisor - 1, bit_timer - 1))

        # Gap between frames.
        gap_cycles = int(self.clk_freq_hz * self.interval_ms / 1000)
        gap_timer  = Signal(range(gap_cycles))

        byte_index = Signal(range(len(self.PATTERN)))
        # start + 8 data + stop, shifted out LSB first. Idle is high, so the
        # register sits at all-ones between frames and the line rests correctly.
        shifter    = Signal(10, init=0x3FF)
        bit_index  = Signal(range(10))

        # Idle high: a UART line at rest is a mark, and Apollo's receiver needs
        # that to recognise the falling start bit.
        m.d.comb += self.pad.o.eq(shifter[0])

        with m.FSM(domain="sync"):

            with m.State("IDLE"):
                m.d.sync += shifter.eq(0x3FF)
                with m.If(~self.stop):
                    with m.If(gap_timer == 0):
                        m.d.sync += [
                            gap_timer  .eq(gap_cycles - 1),
                            byte_index .eq(0),
                        ]
                        m.next = "LOAD"
                    with m.Else():
                        m.d.sync += gap_timer.eq(gap_timer - 1)
                with m.Else():
                    # Reset the gap so that clearing stop transmits promptly
                    # rather than waiting out a stale countdown.
                    m.d.sync += gap_timer.eq(0)

            # Load on a bit boundary, not immediately: entering LOAD mid-bit
            # would overwrite the shifter before the previous stop bit had been
            # held for its full period, truncating one byte and corrupting the
            # next. (Observed in simulation as C1 51 A5 instead of C1 14 01 A5.)
            with m.State("LOAD"):
                with m.If(bit_strobe):
                    with m.Switch(byte_index):
                        for index, value in enumerate(self.PATTERN):
                            with m.Case(index):
                                # start bit low, data LSB first, stop bit high
                                m.d.sync += shifter.eq((1 << 9) | (value << 1))
                    m.d.sync += bit_index.eq(0)
                    m.next = "SHIFT"

            with m.State("SHIFT"):
                with m.If(bit_strobe):
                    m.d.sync += [
                        shifter   .eq(Cat(shifter[1:], 1)),
                        bit_index .eq(bit_index + 1),
                    ]
                    with m.If(bit_index == 9):
                        with m.If(byte_index == len(self.PATTERN) - 1):
                            m.next = "IDLE"
                        with m.Else():
                            m.d.sync += byte_index.eq(byte_index + 1)
                            m.next = "LOAD"

        return m


class ApolloAdvertiserRequestHandler(USBRequestHandler):
    """ Request handler for ApolloAdvertiser. 
    
    Implements default vendor requests related to ApolloAdvertiser.
    """

    """ The bInterfaceProtocol version supported by this request handler. """
    PROTOCOL_VERSION = 0x00

    REQUEST_APOLLO_ADV_STOP = 0xF0

    def __init__(self, if_number, stop_pin):
        super().__init__()
        self.if_number = if_number
        self.stop_pin  = stop_pin

    def elaborate(self, platform):
        m = Module()

        interface         = self.interface
        setup             = self.interface.setup

        #
        # Vendor request handlers.

        with m.If((setup.type == USBRequestType.VENDOR) & \
                  (setup.recipient == USBRequestRecipient.INTERFACE) & \
                  (setup.index == self.if_number)):
            
            with m.If(setup.request == self.REQUEST_APOLLO_ADV_STOP):

                # Notify that we want to manage this request
                m.d.comb += interface.claim.eq(1)

                # Once the receive is complete, respond with an ACK.
                with m.If(interface.rx_ready_for_response):
                    m.d.comb += interface.handshakes_out.ack.eq(1)

                # If we reach the status stage, send a ZLP.
                with m.If(interface.status_requested):
                    m.d.comb += self.send_zlp()
                    m.d.usb += self.stop_pin.eq(1)

        return m
