#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial
from math import *

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

qx = qy = qz = qw = 0.0
yaw_mode = False

def resize((width, height)):
    if height==0:
        height=1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def drawText(position, textString):
    font = pygame.font.SysFont ("Courier", 18, True)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def draw():
    global rquad
    global qx, qy, qz, qw
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity()
    glTranslatef(0,0.0,-7.0)

    # # the way I'm holding the IMU board, X and Y axis are switched
    # # with respect to the OpenGL coordinate system
    # if yaw_mode:                             # experimental
    #     glRotatef(az, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    # else:
    #     glRotatef(0.0, 0.0, 1.0, 0.0)
    # glRotatef(ay ,1.0,0.0,0.0)        # Pitch, rotate around x-axis
    # glRotatef(-1*ax ,0.0,0.0,1.0)     # Roll,  rotate around z-axis

    norm = sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    if(norm == 0):
        return
    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm

    angle = 2 * acos(qw) * (180 / pi)
    x = qx / sqrt(1-(qw*qw))
    y = qy / sqrt(1-(qw*qw))
    z = qz / sqrt(1-(qw*qw))

    latitude = -asin(z)

    if(y**2 + x*2 < 0.0005):
        longitude = 0
    else:
        longitude = atan2(y, x)

    if(longitude < 0):
      longitude += 2 * pi

    latitude *= 180/pi
    longitude *= 180/pi

    osd_text = "pitch: " + str("{0:.2f}".format(latitude)) + ", roll: " + str("{0:.2f}".format(longitude))
    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))
    else:
        osd_line = osd_text

    drawText((-2,-2, 2), osd_line)

    glRotate(angle, y, z, x)

    glBegin(GL_QUADS)
    glColor3f(0.0,1.0,0.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f( 1.0, 0.2, 1.0)

    glColor3f(1.0,0.5,0.0)
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f( 1.0,-0.2,-1.0)

    glColor3f(1.0,0.0,0.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)

    glColor3f(1.0,1.0,0.0)
    glVertex3f( 1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2,-1.0)

    glColor3f(0.0,0.0,1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2, 1.0)

    glColor3f(1.0,0.0,1.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f( 1.0,-0.2,-1.0)
    glEnd()

def read_data():
    global qx, qy, qz, qw
    qx = qy = qz = qw = 0.0
    line_done = 0

    #while not line_done:
    line = ser.readline()
    print line
    angles = line.split()
    if len(angles) == 5:
        qw = float(angles[1])
        qx = float(angles[2])
        qy = float(angles[3])
        qz = float(angles[4])
        line_done = 1

def main():
    global yaw_mode

    video_flags = OPENGL|DOUBLEBUF

    pygame.init()
    screen = pygame.display.set_mode((640,480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize((640,480))
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
        read_data()
        draw()

        pygame.display.flip()
        frames = frames+1

    print "fps:  %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks))
    ser.close()

if __name__ == '__main__': main()
