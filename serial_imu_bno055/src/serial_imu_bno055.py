#!/usr/bin/env python3

import serial
import math
import time

class serialIMU:

	serial.port = serial.Serial()

	def __init__(self, port = "/dev/ttyUSB0"):
		#variable variable imu data
		self.raw_accelerometer = ''
		self.raw_magnetometer = ''
		self.raw_gyroscope = ''
		self.raw_euler = ''
		self.raw_quaternion = ''
		self.raw_linear_acceleration = ''
		self.raw_gravity = ''

		#set serial port serial imu bno055 with arduino
		self.serial_port = serial.Serial(port = port,
										 baudrate = 115200,
										 timeout = 0.1,
										)

	def write_imu(self, data):
		self.serial_port.write(data.encode())

	def read_imu(self):
		response = self.serial_port.readline()
		
		if len(response) != 0:
			response = response.decode()
			if response[0] == '*' and len(response) > 20:
				status = 1
			else:
				status = 0
		else :
			status = 0
		
		return response, status

	def update_imu_data(self):
		response, status = self.read_imu()
		
		if status == 1:
			data = response.split('|')
			self.raw_accelerometer = data[1]
			self.raw_magnetometer = data[2]
			self.raw_gyroscope = data[3]
			self.raw_euler = data[4]
			self.raw_quaternion = data[5]
			self.raw_linear_acceleration = data[6]
			self.raw_gravity = data[7]

	def get_accelerometer(self):
		data_accel = self.raw_accelerometer.split('/')

		try:
			accelerometer_x = float(data_accel[0])
			accelerometer_y = float(data_accel[1])
			accelerometer_z = float(data_accel[2])
		except Exception as ex:
			#print (ex)
			return 0.00, 0.00, 0.00,

		return accelerometer_x, accelerometer_y, accelerometer_z

	def get_magnetometer(self):
		data_mag = self.raw_magnetometer.split('/')

		try:
			magnetometer_x = float(data_mag[0])
			magnetometer_y = float(data_mag[1])
			magnetometer_z = float(data_mag[2])
		except Exception as ex:
			#print (ex)
			return 0.00, 0.00, 0.00,

		return magnetometer_x, magnetometer_y, magnetometer_z

	def get_gyroscope(self):
		data_gyro = self.raw_gyroscope.split('/')

		try:
			gyroscope_x = float(data_gyro[0])
			gyroscope_y = float(data_gyro[1])
			gyroscope_z = float(data_gyro[2])
		except Exception as ex:
			#print (ex)
			return 0.00, 0.00, 0.00,

		return gyroscope_x, gyroscope_y, gyroscope_z

	def get_euler_orientation(self):
		data_euler = self.raw_euler.split('/')

		try:
			euler_x = float(data_euler[0])
			euler_y = float(data_euler[1])
			euler_z = float(data_euler[2])
		except Exception as ex:
			#print (ex)
			return 0.00, 0.00, 0.00,

		return euler_x, euler_y, euler_z

	def get_quaternion_orientation(self):
		data_quaternion = self.raw_quaternion.split('/')

		try:
			quaternion_w = float(data_quaternion[0])
			quaternion_x = float(data_quaternion[1])
			quaternion_y = float(data_quaternion[2])
			quaternion_z = float(data_quaternion[3])
		except Exception as ex:
			#print (ex)
			return 0.00, 0.00, 0.00, 0.00

		return quaternion_w, quaternion_x, quaternion_y, quaternion_z

	def get_linear_acceleration(self):
		data_linear_accel = self.raw_linear_acceleration.split('/')

		try:
			linear_acceleration_x = float(data_linear_accel[0])
			linear_acceleration_y = float(data_linear_accel[1])
			linear_acceleration_z = float(data_linear_accel[2])
		except Exception as ex:
			#print (ex)
			return 0.00, 0.00, 0.00,

		return linear_acceleration_x, linear_acceleration_y, linear_acceleration_z

	def get_gravity(self):
		data_gravity = self.raw_gravity.split('/')

		try:
			gravity_x = float(data_gravity[0])
			gravity_y = float(data_gravity[1])
			gravity_z = float(data_gravity[2])
		except Exception as ex:
			#print (ex)
			return 0.00, 0.00, 0.00,

		return gravity_x, gravity_y, gravity_z

#if __name__ == '__main__':
#	ser_imu = serialIMU();
#	
#	while True:
#		ser_imu.update_imu_data()
#		rp, roll, pitch, yaw = ser_imu.get_quaternion_orientation()
#		print("RP = " + str(rp) + ",Roll = " + str(roll) + ",Pitch = " + str(pitch) + ",Yaw = " + str(yaw));

