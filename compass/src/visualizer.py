#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial

#ser = serial.Serial('/dev/tty.usbserial', 38400, timeout=1)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

ax = ay = az = 0.0
yaw_mode = False

def resize(width, height):
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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax))

    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))
    else:
        osd_line = osd_text

    drawText((-2, -2, 2), osd_line)

    if yaw_mode:
        glRotatef(az, 0.0, 1.0, 0.0)
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(ay, 1.0, 0.0, 0.0)
    glRotatef(-1*ax, 0.0, 0.0, 1.0)

    # Draw rover body
    glBegin(GL_QUADS)
    # Top (Yellow)
    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, 0.3, 0.7)
    glVertex3f(-1.0, 0.3, 0.7)
    glVertex3f(-1.0, 0.3, -0.7)
    glVertex3f(1.0, 0.3, -0.7)
    # Bottom (Purple)
    glColor3f(0.5, 0.0, 0.5)
    glVertex3f(1.0, -0.3, 0.7)
    glVertex3f(-1.0, -0.3, 0.7)
    glVertex3f(-1.0, -0.3, -0.7)
    glVertex3f(1.0, -0.3, -0.7)
    # Front (Red)
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.3, 0.7)
    glVertex3f(-1.0, 0.3, 0.7)
    glVertex3f(-1.0, -0.3, 0.7)
    glVertex3f(1.0, -0.3, 0.7)
    # Back (Green)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.3, -0.7)
    glVertex3f(-1.0, 0.3, -0.7)
    glVertex3f(-1.0, -0.3, -0.7)
    glVertex3f(1.0, -0.3, -0.7)
    # Left (Blue)
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.3, 0.7)
    glVertex3f(-1.0, 0.3, -0.7)
    glVertex3f(-1.0, -0.3, -0.7)
    glVertex3f(-1.0, -0.3, 0.7)
    # Right (Orange)
    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, 0.3, 0.7)
    glVertex3f(1.0, 0.3, -0.7)
    glVertex3f(1.0, -0.3, -0.7)
    glVertex3f(1.0, -0.3, 0.7)
    glEnd()

    # Draw filled wheels
    glColor3f(0.2, 0.2, 0.2)  # Dark gray for wheels
    for x in [-0.8, 0.8]:
        for z in [-0.8, 0.8]:
            glPushMatrix()
            glTranslatef(x, -0.3, z)
            gluDisk(gluNewQuadric(), 0, 0.2, 32, 1)  # Front face of wheel
            gluCylinder(gluNewQuadric(), 0.2, 0.2, 0.1, 32, 1)  # Side of wheel
            glTranslatef(0, 0, 0.1)
            gluDisk(gluNewQuadric(), 0, 0.2, 32, 1)  # Back face of wheel
            glPopMatrix()

def read_data():
    global ax, ay, az
    ax = ay = az = 0.0
    line_done = 0

    # Read the data sent from ESP32
    line = ser.readline()
    angles = line.split(b", ")
    if len(angles) == 3:
        try:
            ay = float(angles[0])
            ax = float(angles[1])
            az = float(angles[2])
            line_done = 1
        except ValueError:
            pass

def main():
    global yaw_mode

    video_flags = OPENGL|DOUBLEBUF
    
    pygame.init()
    screen = pygame.display.set_mode((1366,768), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(1366,768)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  #* quit pygame properly
            break       
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            ser.write(b"z")
        read_data()
        draw()
      
        pygame.display.flip()
        frames = frames+1

    print ("fps:  %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    ser.close()

if __name__ == '__main__': main()