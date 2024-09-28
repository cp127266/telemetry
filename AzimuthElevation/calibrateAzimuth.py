#!/usr/bin/python

from azimuthCalibration import getHeading
from azimuthCalibration import calibrateMag
from azimuthCalibration import turnAZ, turnEL, sendZeroAZ
import serial
import time

#halfAngleOpening = 2

#magMin, magMax = calibrateMag()
#magMin, magMax = ([40000,40000,40000],[-40000,-40000,-40000])

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
ser.reset_input_buffer()

turnAZ(ser, 0) # going back to zero

heading, tiltCompensatedHeading, Acc, gryo, CFangle = \
		getHeading()#magMin = magMin, magMax = magMax)
AccXangle, AccYangle = Acc
gyroXangle, gyroYangle, gyroZang = gryo
CFangleX, CFangley = CFangle

angleToTurnTo = (-tiltCompensatedHeading - 90)%360 # assuming clockwise is positive
	
print("angleToTurnTo: {}".format(angleToTurnTo))
turnAZ(ser, angleToTurnTo) # writing to serial
sendZeroAZ(ser, angleToTurnTo) # writing to serial to set zero

