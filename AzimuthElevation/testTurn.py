#!/usr/bin/python

from azimuthCalibration import turnAZ, turnEL
import sys
import serial

angleAZ = float(sys.argv[1])
angleEL = float(sys.argv[2])

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
ser.reset_input_buffer()

turnAZ(ser, 0)
turnAZ(ser, angleAZ) # writing to serial
turnEL(ser, 0)
turnEL(ser, angleEL)
