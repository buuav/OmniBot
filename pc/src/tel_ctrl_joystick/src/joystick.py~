#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pygame
import math
import time
from time import sleep
import sys


import signal # for catching keyboard interrupt ctrl+c

rospy.init_node('pc', anonymous=True)
#rospy.Subscriber('current',String,current_callback)
pub = rospy.Publisher('joystick',String,queue_size = 14)
pub_vel = rospy.Publisher('command_velocity', Twist, queue_size=10) # publish vel command in standard format

# Exit safely: will run when ctrl+c is pressed
def exit_handler(signal, frame):
    move_command = "@a0000000001?!"
    pub.publish(move_command)
    print 'Program ended!'
    sys.exit(0)

signal.signal(signal.SIGINT, exit_handler) # register the exit interrupt function

#GUI initialization
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)
    def printf(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
    def indent(self):
        self.x += 10
    def unindent(self):
        self.x -= 10
done = False
pygame.init()
clock = pygame.time.Clock()
# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Omni_Wheel Robot")

clock = pygame.time.Clock()
# Initialize the joysticks
pygame.joystick.init()
    
# # Get ready to print
textPrint = TextPrint()

def get_joystick_data():
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
    # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION 
    joystick_count = pygame.joystick.get_count()
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        axes = joystick.get_numaxes()
        for i in range( axes ):
            axis = joystick.get_axis( i )
        velocity=600*math.sqrt(math.pow(math.fabs(joystick.get_axis(0)),2)+math.pow(math.fabs(joystick.get_axis(1)),2))
        velocity=int(velocity/10)*10;
        angle_velocity=450*(math.fabs(joystick.get_axis(2)))
        angle_velocity=int(angle_velocity/10)*10;
        if angle_velocity >800:
            angle_velocity = 800
        elif angle_velocity <40:
            angle_velocity = 0
        if velocity>800:
            velocity=800
        elif velocity<40:
            velocity=0
        if joystick.get_axis(2)>0:
            direction = str(2)
        elif joystick.get_axis(2)<0:
            direction = str(1)
        velocity=str(int(velocity))
        velocity=velocity.zfill(3)
        if joystick.get_axis(2)>0:
            angle_velocity=str(int(angle_velocity))
            angle_velocity=angle_velocity.zfill(3)+'2'
        else:
            angle_velocity=str(int(angle_velocity))
            angle_velocity=angle_velocity.zfill(3)+'1'
        if joystick.get_axis(1)<0:
            angle=(math.atan2(-joystick.get_axis(1),joystick.get_axis(0)))*180/math.pi
            angle=str(int(angle))
            angle=angle.zfill(3)
            move_command='@a'+velocity+angle+angle_velocity+'?!'
        elif joystick.get_axis(1)>0:
            angle=(math.atan2(-joystick.get_axis(1),joystick.get_axis(0)))*180/math.pi+360
            angle=str(int(angle))
            angle=angle.zfill(3)
            move_command='@a'+velocity+angle+angle_velocity+'?!'
        else:
            if joystick.get_axis(0)>0:
                angle = '000'
                move_command='@a'+velocity+angle+angle_velocity+'?!'
            elif joystick.get_axis(0)<0:
                angle = '180'
                move_command='@a'+velocity+angle+angle_velocity+'?!'
  
            elif joystick.get_axis(0) == 0:
                move_command='@a000000'+angle_velocity+'?!'
    pub.publish(move_command)

    # organize vel command in standard format
    vel_mag = float(move_command[2:5])
    vel_dir = float(move_command[5:8])
    omega = float(move_command[8:11])
    if move_command[11] == '2':
        omega=-omega

    twist = Twist()
    twist.linear.x = vel_mag*math.cos(vel_dir*math.pi/180.0) # set velocity
    twist.linear.y = vel_mag*math.sin(vel_dir*math.pi/180.0)
    twist.angular.z = omega        
    pub_vel.publish(twist)  
    
    
current = 0
def current_callback(data):
    global current
    current = data.data
    rospy.loginfo("%s",data.data)

if __name__ == '__main__': 


    while True:
        #screen.fill(WHITE)
        #textPrint.reset()
        get_joystick_data()
        #textPrint.printf(screen, "current = {}".format(current))
        #pygame.display.flip()
        #clock.tick(100)    
        sleep(0.03)
    
       
    
