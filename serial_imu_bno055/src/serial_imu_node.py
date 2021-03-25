#!/usr/bin/env python3

import rospy
import tf
import os
import serial
import time
from serial_imu_bno055 import * 
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

class sensorIMU:

	def __init__(self):
		#init node
		rospy.init_node("ros_serial_imu_node")

		#get node name
		self.node_name = rospy.get_name()

		#get ros params
		self.get_ros_params()

		# Create an IMU instance
		self.serial_imu = serialIMU(port = self.serial_port)

		#internal variables
		self.imu_data_seq_counter = 0
		self.imu_magnetometer_seq_counter = 0
		self.stop_request = False

        #create topics
		self.pub_imu_data = rospy.Publisher('imu/data', Imu, queue_size=1)
		self.pub_imu_magnetometer = rospy.Publisher('imu/magnetometer', MagneticField, queue_size=1)

		# Print node status
		rospy.loginfo(self.node_name + " ready!")

	def get_ros_params(self):
		self.serial_port = rospy.get_param(self.node_name + '/serial_port','/dev/ttyUSB0')
		self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'imu_link')
		self.frequency = rospy.get_param(self.node_name + '/frequency', 100)

	def publish_imu_data(self):
            
		imu_data = Imu()  
        
		quaternion = self.serial_imu.get_quaternion_orientation()
		linear_acceleration = self.serial_imu.get_linear_acceleration()
		gyroscope = self.serial_imu.get_gyroscope()
        
		imu_data.header.stamp = rospy.Time.now()
		imu_data.header.frame_id = self.frame_id
		imu_data.header.seq = self.imu_data_seq_counter

		imu_data.orientation.w = quaternion[0]
		imu_data.orientation.x = quaternion[1]
		imu_data.orientation.y = quaternion[2]
		imu_data.orientation.z = quaternion[3]

		imu_data.linear_acceleration.x = linear_acceleration[0]
		imu_data.linear_acceleration.y = linear_acceleration[1]
		imu_data.linear_acceleration.z = linear_acceleration[2]

		imu_data.angular_velocity.x = gyroscope[0]
		imu_data.angular_velocity.y = gyroscope[1]
		imu_data.angular_velocity.z = gyroscope[2]

		imu_data.orientation_covariance[0] = -1
		imu_data.linear_acceleration_covariance[0] = -1
		imu_data.angular_velocity_covariance[0] = -1

		self.imu_data_seq_counter=+1

		self.pub_imu_data.publish(imu_data)

	def publish_imu_magnetometer(self):

		imu_magnetometer = MagneticField()

		magnetometer = self.serial_imu.get_magnetometer()

		imu_magnetometer.header.stamp = rospy.Time.now()
		imu_magnetometer.header.frame_id = self.frame_id
		imu_magnetometer.header.seq = self.imu_magnetometer_seq_counter

		imu_magnetometer.magnetic_field.x = magnetometer[0]
		imu_magnetometer.magnetic_field.y = magnetometer[1]
		imu_magnetometer.magnetic_field.z = magnetometer[2]

		self.imu_magnetometer_seq_counter=+1

		self.pub_imu_magnetometer.publish(imu_magnetometer)
        		
	def run(self):

        # Set frequency
		rate = rospy.Rate(self.frequency)

		while not rospy.is_shutdown():

            #start_time = time.time()
			try:
				self.serial_imu.update_imu_data()
			except Exception as e:
				continue
            #print("--- %s seconds ---" % (time.time() - start_time)) 

            # Publish imu data
			self.publish_imu_data()
   
            # Publish magnetometer data
			self.publish_imu_magnetometer()

			rate.sleep()


if __name__ == '__main__':

	imu = sensorIMU()

	try:
		imu.run()

	except rospy.ROSInterruptException:
		pass


