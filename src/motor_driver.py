#!/usr/bin/env python

import rospy
import serial
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Motor Speeds:
# right -- [1,64,127]
# left -- [128,192,255]

rightMotorSpeed = 64
leftMotorSpeed = 192
port = serial.Serial('/dev/serial0', 38400)

def twist_cb(msg):
	global rightMotorSpeed, leftMotorSpeed
	
	if msg.angular.z > 0:
		#turn left
		rightMotorSpeed = 64 + int(63*msg.angular.z)
		leftMotorSpeed = 192 + int(63*msg.linear.x)
	elif msg.angular.z < 0:
		#turn right
		rightMotorSpeed = 64 + int(63*msg.linear.x)
		leftMotorSpeed = 192 + int(63*msg.angular.z * -1)
	elif msg.linear.x <= 0:
		#stop
		rightMotorSpeed = 64
		leftMotorSpeed = 192
	elif msg.linear.x > 0:
		#move forward
		rightMotorSpeed = 64 + int(63*msg.linear.x)
		leftMotorSpeed = 192 + int(63*msg.linear.x)
	

	print rightMotorSpeed
	print leftMotorSpeed
	port.write(bytearray([leftMotorSpeed]))
	port.write(bytearray([rightMotorSpeed]))

rospy.init_node('motor_driver')
rospy.Subscriber('cmd_vel', Twist, twist_cb)
rospy.spin()

