#! /usr/bin/env python3

from bmp280 import BMP280
import time

sensor = BMP280(0x77)

while(1):
	sensor.updateValues()
	print(sensor.getTempuratureC())
	print(sensor.getPressureP())
	print(sensor.getPressureA())
	print(sensor.getPressureSeaLevelP())
	print(sensor.getAltitudeM())
	print("-------------------------")
	time.sleep(2)
