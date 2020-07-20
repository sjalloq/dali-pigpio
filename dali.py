#!/usr/bin/env python3

import _dali
import os,sys,time
import pigpio
import atexit

RX = 23
TX = 22

def rx_callback(frame, timestamps):
    #for entry in timestamps:
    #    print(entry)
    print('frame = %s'%hex(frame))

def query_status(tx,addr):
    
    tx.send(0x01A1 | ((addr & 0x3F) << 9))

if __name__ == '__main__':

    pi = pigpio.pi('localhost')
    rx = _dali.rx(pi, RX, rx_callback)
    tx = _dali.tx(pi, TX)

    query_status(tx,0x0)

    time.sleep(1)
    tx.cancel()
    rx.cancel()
    pi.stop()

