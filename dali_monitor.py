#!/usr/bin/env python3

import sys
import time
import pigpio
import _dali
import atexit

HOST='localhost'
LEDS = [17,13,12,16]
RX = 23

def init_leds():
    for led in LEDS:
        pi.write(led,1)      
    for led in LEDS:
        value = pi.read(led)
        print('led %s got value %s' % (led,value))

def cleanup():
    rx.cancel()  
    pi.stop()
    print('cleaned up')

def callback(frame):
    """
    Receives a Dali frame
    """
    print("Dali Monitor: %s" % hex(frame))

pi = pigpio.pi(HOST)
rx = _dali.rx(pi, RX, callback)

atexit.register(cleanup)

init_leds()

while(True):
    pass
