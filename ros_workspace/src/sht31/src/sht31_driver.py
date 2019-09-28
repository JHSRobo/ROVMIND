#!/usr/bin/env python

import rospy
from smbus2 import SMBusWrapper

class SHT31:
	
	##The constructor 
	# @param addr Sets the I2C device address
	def __init__(self, addr=0x44):
		self.address = addr    ## Device address (0x44 default or 0x45)
		self.tempuratureC = 0  ## Tempurature in celcius
		self.tempuratureF = 0  ## Tempurature in fahrenheit
		self.humidity = 0      ## Humidity in % RH (relative humidity)
		self.readError = False ## Status variable for indicating a read error of the SHT31 I2C device
	
	##Updates and decodes the raw sht31 sensor values and stores them into class member variables to be extracted by the getter functions.
	# Must be run before getting sensor values.
	def updateValues(self):

		attempts = 0
		success = False
		with SMBusWrapper(1) as bus:
			while (success != True and attempts < 10):
				# put sensor into signle shot mode with no clock stretching
				try:
					bus.write_word_data(self.address, 0x24, 0x00)
					success = True
				except Exception as e:
					rospy.logerr("Error writting to SHT31 sensor: %s", e)
					attempts += 1
					rospy.sleep(0.25)

			if (attempts == 10):
				rospy.logerr("SHT31-D connection failed! Ending program")
				exit(1)

		rospy.sleep(.015) #experimentally found rest time before reading values
		with SMBusWrapper(1) as bus:
			#read 16 bit temp and humidity with 2 CRC bytes (6 bytes)
			numOfTries = 0
			while(numOfTries < 4):
				try:
					dataBlock = bus.read_i2c_block_data(self.address, 0, 6)
					self.readError = False
					break
				except Exception as e:
					rospy.logerr("SHT31-D read error: %s", e)
					self.readError = True
					numOfTries+=1
					rospy.sleep(.001)
					#print "Retrying..."
			if(not self.readError):
				self.decodeMessage(dataBlock)
			else:
				self.decodeMessage([0,0,0,0,0,0])
				
	## Get the tempurature in degrees celcius
	# @return Tempurature in degrees celcius	
	def getTempuratureC(self):
		return self.tempuratureC
	
	## Get the tempurature in degrees fahrenheit
	# @return Tempurature in degrees fahrenheit	
	def getTempuratureF(self):
		return self.tempuratureF
	
	## Get the humidity in % RH (relative humidity)
	# @return Humidity in % RH	
	def getHumidity(self):
		return self.humidity 
	
	## Calculate the actual data values from the raw sensor data. 
	# Stores the actual data into class member variables.
	# @param raw data from the SHT31	
	def decodeMessage(self, msg):
		rawTemp = (msg[0] << 8) | msg[1]
		rawHum = (msg[3] << 8) | msg[4]
		
		self.humidity = 100 * (rawHum/(65535.0))
		self.tempuratureC = -45 + 175 * (rawTemp/(65535.0))
		self.tempuratureF = -49 + 315 * (rawTemp/(65535.0))


