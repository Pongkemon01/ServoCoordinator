'''
There are 3 ways to show the projection of the block
1) Euler Angles
2) Quaternion Angles
3) Input from raw data


Throughout the code, I have made comments on which lines to comment/uncomment 
depening on which method you choose. 

'''
#Importing modules/libraries 
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
import serial
from pygame.locals import *

from time import sleep
import math
from ctypes import c_float, c_int32, cast, byref, POINTER

#Initializing Variabes 
ax = ay = az = 0.0
t1=0
q0 = float(1.0)
q1 = float(0.0)
q2 = float(0.0)
q3 = float(0.0)
integralFBx = float(0.0)
integralFBy = float(0.0)
integralFBz = float(0.0)

#Initializing USB port connection
myserial = serial.Serial("/dev/tty.usbmodem142103", baudrate=115200, timeout=3.0)

def readlineCR(port):
    rv = ""
    while True:
        ch = (port.read()).decode("utf-8")
        rv += ch
        if ch=='\r' or ch=='\n' or ch=='':
            return rv

#Read Funtion 
def myRead(n):
    global myserial
    myBuffer=''
    while True:
        myBuffer = readlineCR(myserial)
        myBuffer = myBuffer.lstrip().rstrip()
        if len(myBuffer) <= 12:
            continue
        if ("Quaternion:" in myBuffer):  # Found the desired string
            # Split leading "UpdateData: - Quaternion:" from the rest
            quaternion = myBuffer.split(':')[2]
            quaternion = quaternion.lstrip()

            # Split the quaternion into elements
            q_elements = quaternion.split('+')
            if( len(q_elements ) != 4 ):
                continue    # Skip ill-format string

            # Trim all element spaces
            q_elements[0] = q_elements[0].lstrip().rstrip()
            q_elements[1] = q_elements[1].lstrip().rstrip()
            q_elements[2] = q_elements[2].lstrip().rstrip()
            q_elements[3] = q_elements[3].lstrip().rstrip()

            # Check the format and skip ill form
            if ( q_elements[1][len(q_elements[1]) - 1] != 'i' or q_elements[2][len(q_elements[2]) - 1] != 'j' or q_elements[3][len(q_elements[3]) - 1] != 'k' ):
               continue

            # Convert all elements into number
            w = float(q_elements[0])
            x = float(q_elements[1][:len(q_elements[1]) - 1])
            y = float(q_elements[2][:len(q_elements[2]) - 1])
            z = float(q_elements[3][:len(q_elements[3]) - 1])
            break   

    return w, x, y, z

def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


#Funtion to display in GUI 
def drawtext(position, textstring):
    font = pygame.font.SysFont("Courier", 18, True)
    textsurface = font.render(textstring, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textsurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textsurface.get_width(), textsurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

#Function to display the block  
def draw():
    global ax,ay,az
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax))

    osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))

    drawtext((-2, -2, 2), osd_line)

    # the way I'm holding the IMU board, X and Y axis are switched,with respect to the OpenGL coordinate system
    
    az=az+180  #Comment out if reading Euler Angle/Quaternion angles 
    glRotatef(az, 0.0, 1.0, 0.0)      # Yaw, rotate around y-axis

 
    glRotatef(ay, 1.0, 0.0, 0.0)          # Pitch, rotate around x-axis
    glRotatef(-1 * ax, 0.0, 0.0, 1.0)     # Roll, rotate around z-axis

    glBegin(GL_QUADS)
    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


#This function reads the Quaternion angle readings from the BnO055
def ReadQuaterion():
    global ax, ay, az
    w,x,y,z=myRead(2) #calls MyRead function to return after it is written

    #Turns the Quaternion readings into Euler Angles for projection
    ysqr = y*y
    t0 = +2.0 * (w * x + y*z)
    t1 = +1.0 - 2.0 * (x*x + ysqr)
    ax = (math.degrees(math.atan2(t0, t1)))
        
    t2 = +2.0 * (w*y - z*x)
    t2 =  1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    ay= math.degrees(math.asin(t2))
       
    t3 = +2.0 * (w * z + x*y)
    t4 = +1.0 - 2.0 * (ysqr + z*z)
    az=math.degrees(math.atan2(t3, t4))

def main():
    global ax,ay,az
    global yaw_mode
    global serial 
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640, 480)
    frames = 0
    ticks = pygame.time.get_ticks()
    
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
        
        pygame.display.flip()
        frames = frames + 1

        ReadQuaterion()
        draw()
           

if __name__ == '__main__': main()