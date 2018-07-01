#!/usr/bin/env python

import rospy
import serial
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

counter = 0

def twist_cb(msg):
	global counter
	counter += 1
	print counter
	print msg

counter = 0
rospy.init_node('motor_driver')
rospy.Subscriber('cmd_vel', Twist, twist_cb)
rospy.spin()

