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
import pygame
import math
import time
from time import sleep


# Import optitrack functions
import sys
# add the upper directory
sys.path.append("..")
# include optitrack module
from src_optitrack.optitrack import *
from PID import PID

# Global variables
RB_ID = 5 # rigid-body ID in the optitrack system
distance = 1000

opti = OptiTrackInterface()

dest = (5.5, 1.5)
ref_yaw = 0 #set reference yaw = 0
ref_distance = 0.0
distance_output='0'
angle_output='0'
rotate_flag='0'
desire_ang=0
error=0
err=0
def get_vec_ang(vec):
    if (0 == vec[0]): # vx = 0
        if (vec[1] > 0):
	    return 90
	elif (vec[1] < 0):
	    return 270
	else:
	    return 0
    else: # vx != 0
	if(0 == vec[1]): # vy = 0
	    if(vec[0] > 0):
		return 0
	    else:
		return 180
	else: # vx != 0, vy != 0
	    temp = math.fabs(vec[1]/vec[0])
	    if ( vec[0]>0 and vec[1]>0 ): # 1
		return math.atan(temp) * 180 / math.pi+ref_yaw
	    elif ( vec[0]<0 and vec[1]>0 ): # 2
		return (math.pi-math.atan(temp)) * 180 / math.pi
	    elif ( vec[0]<0 and vec[1]<0 ): # 3
		return (math.pi+math.atan(temp)) * 180 / math.pi
	    else:
		return (2*math.pi-math.atan(temp)) * 180 / math.pi

    
def send_distance_PID_command():

    global distance,velocity,angle,distance_output,yaw_degree

    
    if p_1.update(distance)<300 and p_1.update(distance) >0: # maximum speed

    	distance_output=str(int(p_1.update(distance))).zfill(3)

    elif p_1.update(distance) >=300:

    	distance_output='300'
    elif p_1.update(distance) <=0 and p_1.update(distance) > -300:
	distance_output=str(int(-p_1.update(distance))).zfill(3)
    else:
	distance_output='300'
    #distance_output='000'
def send_angle_PID_command():
    global distance,velocity,angle,angle_output,yaw_degree,rotate_flag,err
    if rotate_flag=='2':
 	   err = ref_yaw-yaw_degree
    elif rotate_flag=='1':
	   err = yaw_degree-ref_yaw
    print err
   
    angle_output=str(int(p_2.update(err))) #should be 3 digits, exluding "1" and "2"

    if int(p_2.update(yaw_degree)) > 100:
	angle_output='100'
    elif 0<=int(p_2.update(yaw_degree)) and int(p_2.update(yaw_degree)) <= 100:
	angle_output=str(int(p_2.update(yaw_degree))).zfill(3)
    elif -100<=int(p_2.update(yaw_degree)) and int(p_2.update(yaw_degree)) < 0:
	angle_output=str(int(-p_2.update(yaw_degree))).zfill(3)
    else:
	angle_output='100'

def get_rot_dir( from_ang, to_ang):
    global rotate_flag
    
    if from_ang <0:
		from_ang = from_ang +2*math.pi
    if from_ang> 2*math.pi:
		from_ang = from_ang-2*math.pi
    if to_ang <0:
		to_ang = to_ang +2*math.pi
    if to_ang>2*math.pi:
		to_ang = to_ang -2*math.pi
    if from_ang <=math.pi:
		if to_ang>from_ang and to_ang<=(from_ang+math.pi):
			rotate_flag='1'
		else:
			rotate_flag='2'
    else:
		if to_ang>=(from_ang-math.pi) and to_ang<from_ang:
			rotate_flag='2'
		else:
			rotate_flag='1'



  



current = 0

def current_callback(data):
    global current
    current = data.datals
    rospy.loginfo("%s",data.data)

if __name__ == '__main__': 
    rospy.init_node('pc', anonymous=True)
    #rospy.Subscriber('current',String,current_callback)
    pub = rospy.Publisher('joystick',String,queue_size = 14)
    p_1=PID(3,0.5,0)#position PID
    p_2=PID(0.5,0.05,0)#angle PID
    p_1.setPoint(ref_distance)    #intialize reference distance point
    p_2.setPoint(0)    #intialize reference yaw point
    while(True):
	pos = opti.get_pos(RB_ID)
	yaw_degree=int(pos[5]*180/math.pi)+180#limit Optitrack angle (from_ang) from 0 to 360
	print yaw_degree
	get_rot_dir(yaw_degree*math.pi/180,ref_yaw*math.pi/180)
        distance = math.sqrt((pos[0]-dest[0])*(pos[0]-dest[0]) + (pos[1]-dest[1])*(pos[1]-dest[1]))
	desire_ang = get_vec_ang((dest[0]-pos[0], dest[1]-pos[1]))-ref_yaw
	angle = str(int(desire_ang)).zfill(3)
	send_distance_PID_command()
    	send_angle_PID_command()
	move_command='@a'+distance_output+angle+angle_output+rotate_flag+'?!'

	if distance <= 0.02:
		break
	print move_command
	pub.publish(move_command)
	sleep(0.0001)
    move_command='@a0000000001?!' 
    pub.publish(move_command)
    print "Reach destination!"
    
       
    