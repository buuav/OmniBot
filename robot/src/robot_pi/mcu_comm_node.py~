#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
from time import sleep
from WrapMsg import *
import math

def get_vec_ang(vec):
	'''
	Get the angle of a vector in [0,360).
	Input: a 2D vector
	Output: [0, 360)
	'''
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
			if ( vec[0]>0 and vec[1]>0 ): # 1st quadrant 
				return math.atan(temp) * 180 / math.pi
			elif ( vec[0]<0 and vec[1]>0 ): # 2
				return (math.pi-math.atan(temp)) * 180 / math.pi
			elif ( vec[0]<0 and vec[1]<0 ): # 3
				return (math.pi+math.atan(temp)) * 180 / math.pi
			else: # 4
				return (2*math.pi-math.atan(temp)) * 180 / math.pi

def cmd_vel_callback(data):
	vx = data.linear.x
	vy = data.linear.y
	vel_mag = math.sqrt(vx*vx + vy*vy)
	vel_dir = get_vec_ang((vx, vy))
	omega = data.angular.z
	# print vel_mag, vel_dir, omega
	ser.write(Wrap_Msg_A(vel_mag, vel_dir, omega))
	

def publish_current():
        pub.publish(current)
        rospy.loginfo(current)


if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyAMA0',115200)	
	rospy.init_node('robot', anonymous=True)
        pub = rospy.Publisher('current', String, queue_size=15)
	rospy.Subscriber("command_velocity", Twist, cmd_vel_callback)

 
	while True:
		check=ser.read(2)
	        if check[0]=='@' and check[1]=='1':
			current = ser.read(13)
			if current[12]=='!':
				publish_current()
		sleep(0.001)
    
