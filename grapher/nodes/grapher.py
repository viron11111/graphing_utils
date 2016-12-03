#!/usr/bin/python

import serial
import numpy as np
import rospy
import geometry_msgs.msg 
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Float32
from ms5837.msg import ms5837 
from orientation_estimater.msg import rpy_msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

import binascii
import struct
import encodings
import math
import tf, tf2_ros

class grapher(object):

	def odom(self, data):
		#rospy.logwarn(data.pose.pose.position.x)
		quat = (
			data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quat)
		rospy.logwarn(euler)


	def __init__(self):

		rospy.Subscriber("odom", Odometry, self.odom)
		

		rate = rospy.Rate(20)


		while not rospy.is_shutdown():


			rate.sleep()

def main():
	rospy.init_node('grapher', anonymous=False)

	grapher()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down"
		pass


if __name__ == '__main__':
	main() #sys.argv