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

fb.write( b"!GAM?\n" )
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GAM?\n" )
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GAM?\n" )
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GAM?\n" )
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GAM?\n" )
r = readlineCR( fb )
print("Response = ", r)
fb.write( b"!GAM?\n" )
r = readlineCR( fb )
print("Response = ", r)
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


fb.write( b"!RUN?m=0:dir=cw:spd=100:stp=2000\n")
fb.write( b"!RUN?m=1:dir=ccw:spd=100:stp=2000\n")
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

fb.write( b"!RUN?m=0:dir=ccw:spd=100:stp=2000\n")
fb.write( b"!RUN?m=1:dir=cw:spd=100:stp=2000\n")
r = readlineCR( fb )
print("Response = ", r)

t = input("Pause")

fb.close()
