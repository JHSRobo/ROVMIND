#! /usr/bin/env python3

from sht31 import SHT31
import time

sensor = SHT31(0x44)

while(1):
	sensor.updateValues()
	print(sensor.getTempuratureC())
	print(sensor.getTempuratureF())
	print(sensor.getHumidity())
	print("-------------------------")
	time.sleep(2)
