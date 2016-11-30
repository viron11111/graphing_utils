#!/usr/bin/python

import rospy
import geometry_msgs.msg 
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

import math
import pickle
from time import localtime,strftime
import os
from numpy import linalg,array,amax,amin,add,divide,multiply,subtract,power,Inf,matrix,tile,bmat,random,arange,ones,zeros,eye,squeeze

#I want RPY 

class plot_graphs(object):

	def __init__(self):

		rospy.Subscriber("joy", Joy, self.action)

		rospy.Subscriber("/odometry/filtered", Odometry, self.odom_location)
		
		#desire TF
		self.pose_pub = rospy.Publisher("RC_position", PoseWithCovarianceStamped, queue_size = 1)

		rate = rospy.Rate(20)
		self.start()

		while not rospy.is_shutdown():

			br = tf2_ros.TransformBroadcaster()
			t = geometry_msgs.msg.TransformStamped()

			self.subposx += self.movex
			self.subposy += self.movey

			#this section converts quaternions to euler, applys yaw, then converts euler back to quaternion
			quat = (
				self.subrotx,
				self.subroty,
				self.subrotz,
				self.subrotw)

			euler = tf.transformations.euler_from_quaternion(quat)
			
			turn = euler[2] + self.yaw
			euler = (
				euler[0],
				euler[1],
				turn)
			#rospy.loginfo(euler)

			quat = tf.transformations.quaternion_from_euler(euler[0],euler[1], euler[2])
			self.subrotx = quat[0]
			self.subroty = quat[1]
			self.subrotz = quat[2]
			self.subrotw = quat[3]

			#publish updated desired TF POSE
			t.transform.translation.x = self.subposx
			t.transform.translation.y = self.subposy
			t.transform.translation.z = self.subposz
			t.transform.rotation.x = self.subrotx
			t.transform.rotation.y = self.subroty
			t.transform.rotation.z = self.subrotz
			t.transform.rotation.w = self.subrotw

			#odom = Odometry()

			t.header.stamp = rospy.Time.now()
			t.header.frame_id = "map"
			t.child_frame_id = "desired_position"
			br.sendTransform(t)	

			rate.sleep()

def main():
	rospy.init_node('grapher', anonymous=False)

	plot_graphs()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down"
		pass


if __name__ == '__main__':
	main() #sys.argv