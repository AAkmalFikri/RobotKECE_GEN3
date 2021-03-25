#!/usr/bin/env python3

import serial
import math
import time

class serialOdometry:

	serial.port = serial.Serial()

	def __init__(self, port = "/dev/ttyUSB1"):
		#variable odometry data
		self.raw_odom_x = ''
		self.raw_odom_y = ''
		self.raw_odom_orient = ''

		#set serial port serial stm32f4 master robot
		self.serial_port = serial.Serial(port = port,
										 baudrate = 115200,
										 timeout = 0.1
										)

	def write_serial_odom(self, data):
		self.serial_port.write(data.encode())

	def read_serial_odom(self):
		response = self.serial_port.readline()
		
		if len(response) != 0:
			response = response.decode()
			status = 1
		else :
			status = 0
		
		return response, status

	def update_odom_data(self):
		response, status = self.read_serial_odom()
		
		if status == 1:
			data = response.split('|')
			self.raw_odom_x = data[1]
			self.raw_odom_y = data[2]
			self.raw_odom_orient = data[3]

	def get_odometry(self):
		try:
			odom_x = int(self.raw_odom_x) / 100
			odom_y = int(self.raw_odom_y) / 100
			odom_z = 0.00
			odom_orient = int(self.raw_odom_orient) / 100
			#odom_orient = float(((odom_orient * (-1)) / 180) * math.pi)
		except:
			return 0, 0, 0, 0

		return odom_x, odom_y, odom_z, odom_orient

#if __name__ == '__main__':
#	ser_odom = serialOdometry();
#	
#	while True:
#		ser_odom.serial_port.reset_input_buffer()
#		ser_odom.update_odom_data()
#		x, y, z, orient = ser_odom.get_odometry()
#		print("X = " + str(x) + ", Y = " + str(y) + ", Z = " + str(z) + ", Orient = " + str(orient));

