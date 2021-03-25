#!/usr/bin/env python3

import os
import serial
import time

import rospy
import tf
from serial_odometry import * 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class odometryOmniRobot:

	def __init__(self):
		#init node
		rospy.init_node("ros_serial_odometry_node")

		#get node name
		self.node_name = rospy.get_name()

		#get ros params
		self.get_ros_params()

		#set variabel
		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()
		self.pose_robot = [0, 0, 0, 0]
		self.last_pose_robot = [0, 0, 0, 0]

		# Create an Odometry instance
		self.serial_odometry = serialOdometry(port = self.serial_port)

		#internal variables
		self.stop_request = False

        #create topics
		self.pub_odom_data = rospy.Publisher('odom', Odometry, queue_size=50)
		self.odom_broadcaster = tf.TransformBroadcaster()
		# Print node status
		rospy.loginfo(self.node_name + " ready!")

	def get_ros_params(self):
		self.serial_port = rospy.get_param(self.node_name + '/serial_port','/dev/ttyUSB1')
		self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'base_link')
		self.frequency = rospy.get_param(self.node_name + '/frequency', 100)

	def publish_odom_data(self):
		self.current_time = rospy.Time.now()

		self.pose_robot = self.serial_odometry.get_odometry()

		dt = (self.current_time - self.last_time).to_sec()
		
		vx = (self.pose_robot[0] - self.last_pose_robot[0]) / dt
		vy = (self.pose_robot[1] - self.last_pose_robot[1]) / dt
		vth = (self.pose_robot[3] - self.last_pose_robot[3]) / dt

		odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose_robot[3])
		odom_quat1 = tf.transformations.quaternion_from_euler(0, 0, (self.pose_robot[3] - 1.57))

		self.odom_broadcaster.sendTransform(
				((self.pose_robot[0] * -1), (self.pose_robot[1] * -1), self.pose_robot[2]),
				odom_quat1,
				rospy.Time.now(),
				"base_link",
				"odom"
				)

		odom_data = Odometry()  
		odom_data.header.stamp = self.current_time
		odom_data.header.frame_id = self.frame_id

		odom_data.pose.pose = Pose(Point(self.pose_robot[0], self.pose_robot[1], self.pose_robot[2]), Quaternion(*odom_quat))

		odom_data.child_frame_id = "base_link"
		odom_data.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

		self.pub_odom_data.publish(odom_data)

		self.last_pose_robot = self.pose_robot
		self.last_time = self.current_time


	def run(self):

        # Set frequency
		rate = rospy.Rate(self.frequency)

		while not rospy.is_shutdown():

            #start_time = time.time()
			try:
				self.serial_odometry.update_odom_data()
			except Exception as e:
				continue
            #print("--- %s seconds ---" % (time.time() - start_time)) 

            # Publish imu data
			self.publish_odom_data()
   
			rate.sleep()


if __name__ == '__main__':

	odom = odometryOmniRobot()

	try:
		odom.run()

	except rospy.ROSInterruptException:
		pass


