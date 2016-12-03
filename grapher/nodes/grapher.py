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

import csv
import time
from time import localtime,strftime
import datetime


class grapher(object):

	def odom(self, data):
		#rospy.logwarn(data.pose.pose.position.x)
		quat = (
			data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quat)
		#rospy.logwarn("yaw: %f", euler[2])
		
		self.thruster1_file.writerow([self.time_diff, euler[0], euler[1], euler[2], data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

		self.time_diff = time.time() - self.start_time

	def create_files(self):
		self.start_time = time.time()
		self.time_diff = 0

		date_time = strftime("%d_%m_%y_%H_%M_%S", localtime())
		file_name1 = "/home/andy/catkin_ws/src/graphing_utils/grapher/data/yaw_%s.csv" % date_time
		
		self.thruster1_file = csv.writer(open(file_name1,'w'))
		self.thruster1_file.writerow(["Time", "Roll", "Pitch", "Yaw", "X-Position", "Y-Position", "Z-Position"])

	def __init__(self):

		self.create_files()

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