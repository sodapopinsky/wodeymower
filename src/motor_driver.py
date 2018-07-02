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
	if msg.linear.x == 0:
		rightMotorSpeed = 64
		leftMotorSpeed = 192
	else:
		rightMotorSpeed = 127
		leftMotorSpeed = 255


	print rightMotorSpeed
	print leftMotorSpeed
	port.write(bytearray([leftMotorSpeed]))
	port.write(bytearray([rightMotorSpeed]))

rospy.init_node('motor_driver')
rospy.Subscriber('cmd_vel', Twist, twist_cb)
rospy.spin()

