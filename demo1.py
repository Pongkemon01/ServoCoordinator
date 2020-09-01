#!/Users/Pongkemon/anaconda3/bin/python3

import sys
import serial
import time

print( 'Number of arguments:', len(sys.argv), 'arguments.' )
print( 'Argument List:', str(sys.argv) )

if len(sys.argv) > 1:
    motor_id = int(sys.argv[1])
else:
    motor_id = 0

print( "Motor ID = ", str(motor_id), "\n" )

def readlineCR(port):
    rv = ""
    while True:
        ch = (port.read()).decode("utf-8")
        rv += ch
        if ch=='\r' or ch=='\n' or ch=='':
            return rv

fb = serial.Serial("/dev/tty.usbmodem01", baudrate=115200, timeout=3.0)

fb.write( b"!GAM?\n" )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")


fb.write( b"!CMA?m=0\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!CMA?m=1\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!CMA?m=2\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!CMA?m=3\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!CMA?m=4\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!CMA?m=5\n")
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")


fb.write( b"!GMS?m=0\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GMS?m=1\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GMS?m=2\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GMS?m=3\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GMS?m=4\n")
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GMS?m=5\n")
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")
s = "!SON?m=" + str(motor_id) + "\n";
fb.write( bytes(s, 'utf-8') )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

s = "!GQE?qe=" + str(motor_id) + "\n";
fb.write( bytes(s, 'utf-8') )
r = readlineCR( fb )
print("Response = ", r)
t = input("Pause")

s = "!RUN?m=" + str(motor_id) + ":spd=1000:stp=1000\n";
fb.write( bytes(s, 'utf-8') )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

s = "!GQE?qe=" + str(motor_id) + "\n";
fb.write( bytes(s, 'utf-8') )
r = readlineCR( fb )
print("Response = ", r)
t = input("Pause")

s = "!RUN?m=" + str(motor_id) + ":spd=1000:stp=0\n";
fb.write( bytes(s, 'utf-8') )
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

s = "!GQE?qe=" + str(motor_id) + "\n";
fb.write( bytes(s, 'utf-8') )
r = readlineCR( fb )
print("Response = ", r)
t = input("Pause")

s = "!SOF?m=" + str(motor_id) + "\n";
fb.write( bytes(s, 'utf-8') )
r = readlineCR( fb )
print("Response = ", r)

fb.close()

