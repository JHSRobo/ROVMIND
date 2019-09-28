#! /usr/bin/env python3

import time
from smbus2 import SMBusWrapper

class SHT31:
	def __init__(self, addr):
		self.address = addr
		self.tempuratureC = 0
		self.tempuratureF = 0
		self.humidity = 0
		self.readError = False
		
	def updateValues(self):
		with SMBusWrapper(1) as bus:
			#put sensor into signle shot mode with no clock stretching
			try:
				bus.write_word_data(self.address, 0x24, 0x00)
			except Exception as e:
				print("Error writting to SHT31 sensor: ", e)
				
		
		time.sleep(.012) #experimentally found rest time before reading values
		with SMBusWrapper(1) as bus:
			#read 16 bit temp and humidity with 2 CRC bytes (6 bytes)
			numOfTries = 0
			while(numOfTries < 4):
				try:
					dataBlock = bus.read_i2c_block_data(self.address, 0, 6)
					self.readError = False
					break
				except Exception as e:
					print("SHT31-D read error: ", e)
					self.readError = True
					numOfTries+=1
					time.sleep(.001)
					#print "Retrying..."
			if(not self.readError):
				self.decodeMessage(dataBlock)
			else:
				self.decodeMessage([0,0,0,0,0,0])
				
			
	def getTempuratureC(self):
		return self.tempuratureC
		
	def getTempuratureF(self):
		return self.tempuratureF
	
	def getHumidity(self):
		return self.humidity 
		
	def decodeMessage(self, msg):
		rawTemp = (msg[0] << 8) | msg[1]
		rawHum = (msg[3] << 8) | msg[4]
		
		self.humidity = 100 * (rawHum/(65535.0))
		self.tempuratureC = -45 + 175 * (rawTemp/(65535.0))
		self.tempuratureF = -49 + 315 * (rawTemp/(65535.0))
