#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
from time import sleep

def joystick_callback(data):
    ser.write(data.data)

def publish_current():
        pub.publish(current)
        rospy.loginfo(current)


if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyAMA0',115200)	
	rospy.init_node('robot', anonymous=True)
        pub = rospy.Publisher('current', String, queue_size=15)
	rospy.Subscriber("joystick", String, joystick_callback)


	while True:
		check=ser.read(2)
	        if check[0]=='@' and check[1]=='1':
			current = ser.read(13)
			if current[12]=='!':
				publish_current()
		sleep(0.001)
    
