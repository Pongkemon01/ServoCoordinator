#!/Users/Pongkemon/anaconda3/bin/python3

import sys
import serial
import time

def readlineCR(port):
    rv = ""
    while True:
        ch = (port.read()).decode("utf-8")
        rv += ch
        if ch=='\r' or ch=='\n' or ch=='':
            return rv


fb = serial.Serial("/dev/tty.usbmodem01", baudrate=115200, timeout=3.0)

t = input("Pause")

print("1")
fb.write( b"!HME?\n" )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

print("2")
fb.write( b"!HME?\n" )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

print("3")
fb.write( b"!HME?\n" )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

print("4")
fb.write( b"!HME?\n" )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

print("5")
fb.write( b"!HME?\n" )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

print("6")
fb.write( b"!HME?\n" )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

fb.close()

