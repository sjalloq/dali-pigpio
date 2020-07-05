#!/usr/bin/env python3

import time
import pigpio


class rx():
    """
    A class to read a Dali frame.
    """

    TE      = 834/2  # half bit time = 417 usec
    MIN_TE  = 350
    MAX_TE  = 490
    MIN_2TE = 760
    MAX_2TE = 900
    STP_2TE = 1800

    def __init__(self, pi, gpio, callback=None, glitch=150):
        """
        User must pass in the pigpio.pi object, the gpio number to use as the receive pin
        and a callback method to be called on every received frame.
        """
        self.pi     = pi
        self.gpio   = gpio
        self.cb     = callback
        self.glitch = glitch

        self._in_code       = 0
        self._edges         = 0
        self._code          = 0
        self._prev_edge_len = 0
        
        pi.set_mode(gpio, pigpio.INPUT)
        pi.set_glitch_filter(gpio, glitch)

        self._last_edge_tick = pi.get_current_tick()
        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbe)

    def cancel(self):
        """
        Called on program exit to cleanup.
        """
        if self._cb is not None:
            self.pi.set_glitch_filter(self.gpio, 0) # Remove glitch filter.
            self.pi.set_watchdog(self.gpio, 0) # Cancel the watchdog
            self._cb.cancel()
            self._cb = None
            self.pi.stop()

    def _wdog(self,milliseconds):
        """
        Starts a watchdog timer on the RX pin that fires after waiting for
        the defined number of microseconds.

        The watchdog can be cancelled before it has fired by calling this
        method with a value of 0.
        """
        self.pi.set_watchdog(self.gpio, milliseconds)

    def _stop(self):
        """
        Called at the end of a received frame after the stop bits have been
        detected.  The user callback is called with the received frame as a
        parameter.
        """
        self.frame  = self._code
        self._edges = 0
        self._code  = 0
        if self.cb is not None:
            self.cb(self.frame)

    def _decode(self,high_time,low_time):
        """
        Called on every rising edge of the Dali bus, this method decodes
        the bits received since the last call.
        
        In order to decode the received bits, the previous half bit state 
        is stored in self._prev.  Using this in conjunction with the length
        of the high and low periods passed in as arguments forms a 3-bit
        encoded word.  

        The previous bit value is left shifted by 2 and the 2 LSBs are 
        constructed by checking if the high and low pulses were TE or 2TE
        long, i.e. a half bit width or a full bit width.  This gives the
        concatenation as:
            { self._prev, low_time == 2TE, high_time == 2TE }

        In the following example, after the start bit is received, we see
        3'b101 in the MSBs of the transmitted frame.  Each rising edge of
        the waveform is discussed below.

        1.  The first rising edge belongs to the Start bit.  We ignore this
            outside of setting the initial _prev value to 1.
        2.  At the second rising edge, two short pulses have been received
            giving an encoded word of 3'b100.  The received bit is 1'b1 and
            the _prev value is set to 1.
        3.  The third rising edge follows two long pulses giving 3'b111.
            We have received two bits, {0,1} since the last posedge and
            the _prev value is again updated to 1.


            . start .  n-1  .  n-2  .  n-3  .
            .       .       .       .       .
        ----+   +---+   +---+---+   .   +---.
            |   |   |   |   .   |   .   |   .
            +---+   +---+   .   +---+---+   .
            .       .       .       .       .
            .  "S"  .  "1"  .  "0"  .  "1"  .


        These timings give rise to the truth table shown below.

         Previous      Long       Long                     New     
         Half Bit  | Low Time | High Time |   Action   | Half Bit
       ------------+----------+-----------+------------+-------------
            0      |    0     |     0     |  Shift 0   |   0
            0      |    0     |     1     |  -ERROR-   |   *
            0      |    1     |     0     |  Shift 0,1 |   1
            0      |    1     |     1     |  -ERROR-   |   *
            1      |    0     |     0     |  Shift 1   |   1
            1      |    0     |     1     |  Shift 0   |   0
            1      |    1     |     0     |  -ERROR-   |   *
            1      |    1     |     1     |  Shift 0,1 |   1
        """
        action = self._prev << 2
        
        if high_time > self.MIN_2TE and high_time < self.MAX_2TE:
            action = action | 1
        elif not (high_time > self.MIN_TE and high_time < self.MAX_TE):
            self._in_code = 0
            pass

        if low_time > self.MIN_2TE and low_time < self.MAX_2TE:
            action = action | 2
        elif not (low_time > self.MIN_TE and low_time < self.MAX_TE):
            self._in_code = 0
            pass

        self._code = self._code << 1
                
        if action in [1,3,6]:
            self._in_code = 0
            pass
        else:
            if action in [2,7]:
                self._code = self._code << 1
                self._code += 1
                self._prev = 1
            else:
                if action == 4:
                    self._code += 1
                    self._prev = 1
                else:
                    self._prev = 0

    def _cbe(self, gpio, level, tick):
        """
        Called on either rising or falling edge of the gpio, the time
        since the last edge is recorded and a pair of value are passed
        to the decode method on each rising edge.  A 2 millisecond 
        watchdog is used to signal the end of the frame.
        """

        # Disable the watchdog
        self._wdog(0)

        if level < 2:
            # Received an edge interrupt
            edge_len = pigpio.tickDiff(self._last_edge_tick, tick)
            self._last_edge_tick = tick

            if self._edges < 2:
                # Start bit
                self._prev = 1
            else:
                if self._edges % 2:
                    # Rising edge; decode the low/high time
                    self._decode(self._prev_edge_len, edge_len)
                else:
                    # Falling edge; capture the high time
                    self._prev_edge_len = edge_len
            self._edges += 1

            # Set the watchdog to a time equivalent to 2 stop bits
            self._wdog(round(self.STP_2TE/1000.0))
        else:
            # Received watchdog timeout so end of frame
            self._stop()


class tx():
    """
    A class to transmitt a Dali frame.
    """
    def __init__(self, pi, gpio, gap=9000, te=417):
        self.pi = pi
        self.gpio = gpio
        self.gap = gap
        self.te = te
        self.tstop = 4*te

        self._make_waves()

        self.pi.set_mode(gpio, pigpio.OUTPUT)
        self.pi.set_pull_up_down(gpio, pigpio.PUD_OFF)

    def _make_waves(self):
        """
        Generate the basic '1' and '0' Manchester encoded waveforms.
        """
        wf = []
        wf.append(pigpio.pulse(0, 1<<self.gpio, self.te))
        wf.append(pigpio.pulse(1<<self.gpio, 0, self.te))
        self.pi.wave_add_generic(wf)
        self._start = self.pi.wave_create()
        
        wf = []
        wf.append(pigpio.pulse(1<<self.gpio, 0, self.tstop))
        self.pi.wave_add_generic(wf)
        self._stop = self.pi.wave_create()
        
        wf = []
        wf.append(pigpio.pulse(1<<self.gpio, 0, self.te))
        wf.append(pigpio.pulse(0, 1<<self.gpio, self.te))
        self.pi.wave_add_generic(wf)
        self._wid0 = self.pi.wave_create()

        wf = []
        wf.append(pigpio.pulse(0, 1<<self.gpio, self.te))
        wf.append(pigpio.pulse(1<<self.gpio, 0, self.te))
        self.pi.wave_add_generic(wf)
        self._wid1 = self.pi.wave_create()

    def send(self, code, bits=16, repeats=1):
        """
        Transmits a Dali frame.
        """
        chain = [255, 0, self._start]

        bit = (1<<(bits-1))
        for i in range(bits):
            if code & bit:
                chain += [self._wid1]
            else:
                chain += [self._wid0]
            bit = bit >> 1

        chain += [self._stop, 255, 1, repeats, 0]

        self.pi.wave_chain(chain)

        while self.pi.wave_tx_busy():
            time.sleep(0.1)

    def cancel(self):
        """
        Cancels the Dali transmitter.
        """
        self.pi.wave_delete(self._start)
        self.pi.wave_delete(self._stop)
        self.pi.wave_delete(self._wid0)
        self.pi.wave_delete(self._wid1)
        self.pi.stop()


if __name__ == '__main__':

    import sys
    import time
    import pigpio
    import _dali
    import atexit
    import argparse

    def callback(frame):
        print('Dali frame = %s'%hex(frame))

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', help='specify the hostname', default='localhost')
    args = parser.parse_args()

    LEDS = [17,13,12,16]
    RX = 23
    TX = 22

    pi = pigpio.pi(args.host)
    rx = _dali.rx(pi, RX, callback)

    def init_leds():
        for led in LEDS:
            pi.write(led,1)      

    atexit.register(rx.cancel)

    init_leds()

    while(True):
        time.sleep(20)
