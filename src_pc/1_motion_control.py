'''
	Script for PID control on position and angle of the Omni-directional Robot.
	Author:: Zijian Wang <zjwang@bu.edu>
'''
'''
    Copyright (C) 2015  Zijian Wang <zjwang@bu.edu>
    Multi-Robot Systems Laboratory (MSL), Boston University

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import rospy
from std_msgs.msg import String
import math
from time import sleep
from timeit import default_timer as timer
import signal # for catching keyboard interrupt ctrl+c
from PID import PID

# Import optitrack functions
import sys
sys.path.append("..") # add the upper directory
from src_optitrack.optitrack import * # include optitrack module


# ROS Initialization 
rospy.init_node('pc', anonymous=True)
pub = rospy.Publisher('joystick',String,queue_size = 14)    #send command data to robot

# Create Optitrack Interface
opti = OptiTrackInterface()
RB_ID = 1 # rigid-body ID in the optitrack system

# Exit safely: will run when ctrl+c is pressed
def exit_handler(signal, frame):
	move_command = "@a0000000001?!"
	pub.publish(move_command) # stop the robot
	print 'Program ended!'
	sys.exit(0)

signal.signal(signal.SIGINT, exit_handler) # register the exit interrupt function


#########################################################
############### Global Geometry Functions ###############
#########################################################
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

def get_rot_dir_str( from_ang, to_ang):
	'''
	Determine the shortest direction to ratotate from the from_ang to to_ang
	Input: [0,360)
	Output: a String, '1' -> rotate CCW, '2' -> rotate CW
	'''
	from_ang = from_ang*math.pi/180
	to_ang = to_ang*math.pi/180

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
			return '1'
		else:
			return '2'
	else:
		if to_ang>=(from_ang-math.pi) and to_ang<from_ang:
			return '2'
		else:
			return '1'

def get_inter_ang(ang1, ang2):
	'''
	Get the intersection angle between two angles.
	Input: ang1, ang2, in degree, [0,360)
	Output: in degree, [0, 180)
	'''
	dif = ang1 - ang2
	if dif < - 180:
		ret = dif + 2 * 180
	elif dif >= 180:
		ret = dif - 2 * 180
	else:
		ret = dif

	return math.fabs(ret)		


#########################################################
################## Robot Motion Control #################
#########################################################
def bot_move_to_pos(opti, motion_pub, dest, desire_robot_heading, StopAfterReach = True):
	'''
	Move the robot to a destination, while maintaining its yaw angle @ desire_robot_heading
	Input:
		opti: OptiTrackInterface
		motion_pub: ROS publisher for sending velocity commands
		dest: 2D vector, destination
		desire_robot_heading: [0, 360), negative angles (-180,0) may work
		StopAfterReach: stop the robot or not when the destination is reached
	'''
	hPID_position = PID(10,0.1,0) # PID instance for position control. 
	hPID_position.setPoint(0)   

	hPID_angle = PID(0.4,0.002,0) # PID instance for yaw angle control. 
	hPID_angle.setPoint(0)

	distance = 10000 # initialize the distance as a very large value
    
	while(distance > 0.05):
		# read position from optitrack
		pos = opti.get_pos(RB_ID) # (x,y,z,pitch,roll,yaw)	
		yaw = pos[5]*180/math.pi  # in degree, [-180,180]

		# Control for linear velocity
		distance = math.sqrt((pos[0]-dest[0])*(pos[0]-dest[0]) + (pos[1]-dest[1])*(pos[1]-dest[1]))	#distance between center of body to the destination
		desire_ang_w = get_vec_ang((dest[0]-pos[0], dest[1]-pos[1]))	# in degree [0,360). in world frame
		desire_ang_b = desire_ang_w - yaw # in body frame
		if desire_ang_b < 0:
			desire_ang_b = desire_ang_b + 360
		if desire_ang_b > 360:
			desire_ang_b = desire_ang_b - 360

		if distance > 0.3: # if far from the destination, move in full speed
			vel_mag_pid = 200
		else: # if close to destination, use PID control to achive high accuracy
			vel_mag_pid = 50*math.fabs(hPID_position.update(distance))
		if vel_mag_pid > 200: # limit the speed
			vel_mag_pid = 200
		
		vel_str_cmd = str(int(vel_mag_pid)).zfill(3) 
		vel_ang_str_cmd = str(int(desire_ang_b)).zfill(3)

		# Control for angular velocity
		rot_dir = get_rot_dir_str(yaw,desire_robot_heading) # rotation direction

		dif_ang = get_inter_ang(yaw, desire_robot_heading) 
		omega_pid = math.fabs(hPID_angle.update(dif_ang))
		if omega_pid > 200:
			omega_pid = 200
		omega_str_cmd = str(int(omega_pid)).zfill(3)

		# synthesize command and publish command to robot
		move_command='@a'+vel_str_cmd+vel_ang_str_cmd+omega_str_cmd+rot_dir+'?!'
		motion_pub.publish(move_command)
		
		sleep(0.001)

	# Action after reaching destination
	if(StopAfterReach):
		move_command = "@a0000000001?!"
		motion_pub.publish(move_command) # stop the robot
	print 'Reach Destination', dest, '. Function Exited.'


    
#########################################################
##################### Main Function #####################
#########################################################
if __name__ == '__main__': 
	
	dest1 = (1,2)
	dest2 = (4,1)

	bot_move_to_pos(opti, pub, dest2, 45)

	sys.exit(0)