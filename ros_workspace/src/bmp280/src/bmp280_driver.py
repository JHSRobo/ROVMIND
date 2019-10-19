#!/usr/bin/env python

import rospy
from smbus2 import SMBusWrapper

#recommended sampling frequency = 3
#x16 oversampling 

## BMP280 default address (0x77).
BMP280_I2CADDR           = 0x77

# BMP280 Temperature Registers
BMP280_REGISTER_DIG_T1 = 0x88
BMP280_REGISTER_DIG_T2 = 0x8A
BMP280_REGISTER_DIG_T3 = 0x8C

# BMP280 Pressure Registers
BMP280_REGISTER_DIG_P1 = 0x8E
BMP280_REGISTER_DIG_P2 = 0x90
BMP280_REGISTER_DIG_P3 = 0x92
BMP280_REGISTER_DIG_P4 = 0x94
BMP280_REGISTER_DIG_P5 = 0x96
BMP280_REGISTER_DIG_P6 = 0x98
BMP280_REGISTER_DIG_P7 = 0x9A
BMP280_REGISTER_DIG_P8 = 0x9C
BMP280_REGISTER_DIG_P9 = 0x9E

BMP280_REGISTER_CONTROL = 0xF4
BMP280_REGISTER_CONFIG = 0xF5
#Pressure measurments
BMP280_REGISTER_PRESSUREDATA_MSB = 0xF7
BMP280_REGISTER_PRESSUREDATA_LSB = 0xF8
BMP280_REGISTER_PRESSUREDATA_XLSB = 0xF9
#Temperature measurments
BMP280_REGISTER_TEMPDATA_MSB = 0xFA
BMP280_REGISTER_TEMPDATA_LSB = 0xFB
BMP280_REGISTER_TEMPDATA_XLSB = 0xFC

# Commands
BMP280_READCMD = 0x3F

#Always read tempurature first then pressure in main program!!!

class BMP280:
	
	##The constructor 
	# @param addr Sets the I2C device address
	def __init__(self, addr=0x77):
		self.address = addr

		#calibration values 
		calibrationSuccess = False 
		attempts = 0
		self.tfine = 0
		while(calibrationSuccess == False and attempts < 10):
			try:
				self.loadCalibration()
				with SMBusWrapper(1) as bus:
					try:
						#put sensor into normal mode - ultra high resolution
						bus.write_byte_data(self.address, BMP280_REGISTER_CONTROL, 0xFF)
						bus.write_byte_data(self.address, BMP280_REGISTER_CONFIG, 0x1C)
					except Exception as e:
						rospy.logerr("Error writting mode and settings to BMP280 sensor: %s", e)
				calibrationSuccess = True 
			except Exception as e:
				attempts += 1
				rospy.logerr("Error reading BMP280 calibration data: %s", e)
				rospy.sleep(0.25)

		if(attempts == 10):
			rospy.logerr("BMP280 failed program closing!")
			exit()
	##Updates the raw bmp280 sensor values and stores them into class member variables to be extracted by the getter functions.
	# Must be run before getting sensor values.
	def updateValues(self):

		with SMBusWrapper(1) as bus:
			#read 16 bit temp and humidity with 2 CRC bytes (6 bytes)
			numOfTries = 0
			while(numOfTries < 4):
				try:
					#Read raw tempurature data
					msb = bus.read_byte_data(self.address, BMP280_REGISTER_TEMPDATA_MSB)
					lsb = bus.read_byte_data(self.address, BMP280_REGISTER_TEMPDATA_LSB)
					xlsb = bus.read_byte_data(self.address, BMP280_REGISTER_TEMPDATA_XLSB)
					self.rawTemp = ((msb << 8 | lsb) << 8 | xlsb) >> 4
					#Read raw pressure data
					msb = bus.read_byte_data(self.address, BMP280_REGISTER_PRESSUREDATA_MSB)
					lsb = bus.read_byte_data(self.address, BMP280_REGISTER_PRESSUREDATA_LSB)
					xlsb = bus.read_byte_data(self.address, BMP280_REGISTER_PRESSUREDATA_XLSB)
					self.rawPressure = ((msb << 8 | lsb) << 8 | xlsb) >> 4

					self.readError = False
					break
				except Exception as e:
					rospy.logerr("BMP280 read error: %s", e)
					self.readError = True
					numOfTries+=1
					rospy.sleep(.001)
					#print "Retrying..."
			if(self.readError):
				self.rawTemp = 0
				self.rawPressure = 0

	##Gets the absolute tempurature in celcius 
	# @return Tempurature in celcius
	def getTempuratureC(self):
		TMP_PART1 = (((self.rawTemp>>3) - (self.cal_REGISTER_DIG_T1<<1)) * self.cal_REGISTER_DIG_T2) >> 11
		TMP_PART2 = (((((self.rawTemp>>4) - (self.cal_REGISTER_DIG_T1)) * ((self.rawTemp>>4) - (self.cal_REGISTER_DIG_T1))) >> 12) * (self.cal_REGISTER_DIG_T3)) >> 14
		TMP_FINE = TMP_PART1 + TMP_PART2
		## Value needed for tempurature compensation on pressure and altitude values
		self.tfine = TMP_FINE
		self.temp = ((TMP_FINE*5+128)>>8)/100.0
		return self.temp
	
	##Gets the absolute compensated pressure in pascal
	# @return Pressure in pascal
	def getPressureP(self):
		#for pressure calculation we need a temperature, checking if we have one, and reading data if not
		self.getTempuratureC()
            
		var1 = self.tfine - 128000
		var2 = var1 * var1 * self.cal_REGISTER_DIG_P6
		var2 = var2 + ((var1*self.cal_REGISTER_DIG_P5)<<17);
		var2 = var2 + ((self.cal_REGISTER_DIG_P4)<<35);
		var1 = ((var1 * var1 * self.cal_REGISTER_DIG_P3)>>8) + ((var1 * self.cal_REGISTER_DIG_P2)<<12);
		var1 = ((((1)<<47)+var1))*(self.cal_REGISTER_DIG_P1)>>33;

		if var1 == 0:
			return 0

		p = 1048576 - self.rawPressure;
		p = int((((p<<31) - var2)*3125) / var1);
		var1 = ((self.cal_REGISTER_DIG_P9) * (p>>13) * (p>>13)) >> 25;
		var2 = ((self.cal_REGISTER_DIG_P8) * p) >> 19;

		p = ((p + var1 + var2) >> 8) + ((self.cal_REGISTER_DIG_P7)<<4);
		self.pressure = p / 256.0

		#return pressure in pascals
		return self.pressure;

	##Gets the absolute compensated pressure in atm
	# @return Pressure in atm
	def getPressureA(self):
		return self.getPressureP()/101325
		

	##Gets the relative compensated pressure in pascal with relation to a known altitude
	# @param altitude_m Known altitude in meters for a relative pressure setting 
	# @return Relative pressure in pascal
	def getPressureSeaLevelP(self, altitude_m=0.0):
		#Calculates the pressure at sealevel when given a known altitude in meters. Returns a value in Pascals.
		self.getPressureP()
		pressure = float(self.pressure)
		p0 = pressure / pow(1.0 - altitude_m/44330.0, 5.255)
		return p0 

	##Gets the relative altitude in meters with relation to pressure at sealevel
	# @param sealevel_pa Known pressure in pascal for a relative altitude setting
	# @return Relative altitude in meters
	def getAltitudeM(self, sealevel_pa=101325.0):
		self.getPressureP()
		altitude = 44330.0 * (1.0 - pow(self.pressure / sealevel_pa, (1.0/5.255)))
		#return altitude in meters
		return altitude 
		
	##Loads factory calibration data set on the BMP280
	def loadCalibration(self):
		with SMBusWrapper(1) as bus:
			self.cal_REGISTER_DIG_T1 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_T1) # UINT16
			self.cal_REGISTER_DIG_T2 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_T2) # INT16
			self.cal_REGISTER_DIG_T3 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_T3) # INT16
			self.cal_REGISTER_DIG_P1 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P1) # UINT16
			self.cal_REGISTER_DIG_P2 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P2) # INT16
			self.cal_REGISTER_DIG_P3 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P3) # INT16
			self.cal_REGISTER_DIG_P4 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P4) # INT16
			self.cal_REGISTER_DIG_P5 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P5) # INT16
			self.cal_REGISTER_DIG_P6 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P6) # INT16
			self.cal_REGISTER_DIG_P7 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P7) # INT16
			self.cal_REGISTER_DIG_P8 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P8) # INT16
			self.cal_REGISTER_DIG_P9 = bus.read_word_data(self.address, BMP280_REGISTER_DIG_P9) # INT16
