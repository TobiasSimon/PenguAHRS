
# OpenGL AHRS visualization
#
# Copyright (C) 2012 Tobias Simon
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.



from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import pygame
from sys import exit, argv
from gltexture import Texture
from math import *


def quat2euler(w, x, y, z):
   pitch = atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z)
   roll = atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z)
   yaw = asin(2 * x * y + 2 * z * w)
   return yaw, pitch, roll

def get_event():
   for event in pygame.event.get():
      if event.type == pygame.QUIT:
         exit()

def reshape((width,height)):
   glViewport(0, 0, width, height)
   glMatrixMode(GL_PROJECTION)
   glLoadIdentity()
   gluPerspective(40, 1.0*width/height, 0.1, 1000.0)
   glMatrixMode(GL_MODELVIEW)

def init():
   global screen
   screen = (800, 600)
   glutInit(argv)
   pygame.init()
   surface = pygame.display.set_mode(screen, pygame.OPENGL|pygame.DOUBLEBUF, 16)
   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
   glClearColor(1.0, 1.0, 1.0, 1.0)
   glDisable(GL_LIGHTING) 
   glEnable(GL_CULL_FACE)
   global tex
   tex = Texture('texture.bmp')
   global quadratic
   quadratic = gluNewQuadric()
   gluQuadricNormals(quadratic, GLU_SMOOTH)
   reshape(screen)


def text_2d(value, x, y):
   glMatrixMode(GL_PROJECTION);
   matrix = glGetDouble(GL_PROJECTION_MATRIX)
   glLoadIdentity();
   glOrtho(0.0, screen[1] or 32, 0.0, screen[0] or 32, -1.0, 1.0)
   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glLoadIdentity();
   glRasterPos2i(x, y);
   line = 0
   for character in value:
      if character == '\n':
         line += 1
         glRasterPos2i(x, y-(line * 22))
      else:
         glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(character))
   glPopMatrix()
   glMatrixMode(GL_PROJECTION)
   glLoadMatrixd(matrix)
   glMatrixMode(GL_MODELVIEW)


def draw_arrow(length, tip = True):
   glPushMatrix()
   glLineWidth(3)
   glBegin(GL_LINES)
   glVertex3f(0, 0, 0)
   glVertex3f(length, 0, 0)
   glEnd()
   if tip:
      glTranslatef(length, 0, 0)
      glRotatef(90, 0, 1, 0)
      gluCylinder(quadratic, 0.05, 0.0, 0.3, 16, 16)
      gluSphere(quadratic, 0.05, 32, 32);
   glPopMatrix()


def draw():
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
   glLoadIdentity()
   gluLookAt(0,    6,  0,
             0, -100,  0,
             0,    0, -1)

   glRotate(20.0, 1, 0, 0)
   glRotate(220.0, 0, 0, 1)
   
   # draw fixed coordinate system:
   glPushMatrix()
   glColor3f(0.5, 0.0, 0.0)
   text_2d("E", 470, 500)
   glPushMatrix()
   glRotatef(90, 0, 0, 1)
   draw_arrow(3)
   glPopMatrix()
   glColor3f(0.0, 0.0, 0.5)
   text_2d("N", 80, 500)
   glPushMatrix()
   glRotatef(90, 1, 0, 0)
   draw_arrow(3)
   glPopMatrix()
   glColor3f(0.0, 0.5, 0.0)
   text_2d("D", 280, 100)
   glPushMatrix()
   glRotatef(-90, 0, 1, 0)
   draw_arrow(2)
   glPopMatrix()
   glPopMatrix()
   
   if quat:
      # draw fixed coordinate system:
      glPushMatrix()
      glColor3f(0.5, 0.0, 0.0)
      glPushMatrix()
      glRotatef(90, 0, 0, 1)
      draw_arrow(world_acc[0], False)
      glPopMatrix()
      glColor3f(0.0, 0.0, 0.5)
      glPushMatrix()
      glRotatef(90, 1, 0, 0)
      draw_arrow(world_acc[1], False)
      glPopMatrix()
      glColor3f(0.0, 0.5, 0.0)
      glPushMatrix()
      glRotatef(-90, 0, 1, 0)
      draw_arrow(world_acc[2], False)
      glPopMatrix()
      glPopMatrix()
      print world_acc
      
      # rotate using AHRS quaternion:
      glRotate(360.0 * acos(quat[0]) / pi, quat[1], quat[2], quat[3])
      
      # draw virtual AHRS:
      glEnable(GL_TEXTURE_2D)
      h = 0.02
      w = 0.8
      tex.bind()
      glBegin(GL_QUADS)
      glTexCoord2f(0.0, 0.0); glVertex3f(-w, -w,  h)
      glTexCoord2f(0.5, 0.0); glVertex3f( w, -w,  h)
      glTexCoord2f(0.5, 1.0); glVertex3f( w,  w,  h)
      glTexCoord2f(0.0, 1.0); glVertex3f(-w,  w,  h)
      
      glTexCoord2f(1.0, 0.0); glVertex3f(-w, -w, -h)
      glTexCoord2f(1.0, 1.0); glVertex3f(-w,  w, -h)
      glTexCoord2f(0.5, 1.0); glVertex3f( w,  w, -h)
      glTexCoord2f(0.5, 0.0); glVertex3f( w, -w, -h)
      
      glVertex3f(-w,  w, -h)
      glVertex3f(-w,  w,  h)
      glVertex3f( w,  w,  h)
      glVertex3f( w,  w, -h)
      
      glVertex3f(-w, -w, -h)
      glVertex3f( w, -w, -h)
      glVertex3f( w, -w,  h)
      glVertex3f(-w, -w,  h)

      glVertex3f( w, -w, -h)
      glVertex3f( w,  w, -h)
      glVertex3f( w,  w,  h)
      glVertex3f( w, -w,  h)

      glVertex3f(-w, -w, -h)
      glVertex3f(-w, -w,  h)
      glVertex3f(-w,  w,  h)
      glVertex3f(-w,  w, -h)
      glEnd()
      
      glDisable(GL_TEXTURE_2D)

      # draw arrows:
      glColor3f(1.0, 0.0, 0.0)
      glPushMatrix()
      glRotatef(90, 0, 0, 1)
      draw_arrow(1.5)
      glPopMatrix()
      glColor3f(0.0, 0.0, 1.0)
      glPushMatrix()
      glRotatef(90, 1, 0, 0)
      draw_arrow(1.5)
      glPopMatrix()
      glColor3f(0.0, 1.0, 0.0)
      glPushMatrix()
      glRotatef(-90, 0, 1, 0)
      draw_arrow(1.5)
      glPopMatrix()
    
      # draw text:
      glColor3f(0.0, 0.0, 0.0)
      euler = quat2euler(*quat)
      text_2d("Roll:\t%.1f deg\nPitch:\t%.1f deg\nYaw:\t%.1f deg" % (euler[2] * 180 / pi, euler[1] * 180 / pi, euler[0] * 180 / pi), 100, 100)
   pygame.display.flip()


world_acc = [0, 0, 0]
quat = None


def read_input():
   import socket
   UDP_IP = "127.0.0.1"
   UDP_PORT = 5005
   sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   sock.bind((UDP_IP, UDP_PORT))
   global quat, world_acc
   while True:
      try:
         line, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
         array = map(float, line.split(' '))
         quat = array[0:4]
         world_acc = array[4:7]
      except:
         pass


def main():
   from threading import Thread
   t = Thread(target = read_input)
   t.daemon = True
   t.start()
   init()
   maxfps = 30
   clock = pygame.time.Clock()
   while True:
      clock.tick(maxfps)
      draw()
      get_event()

if __name__ == "__main__":
   try:
      main()
   except:
      pass

