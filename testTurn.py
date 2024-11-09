#!/usr/bin/python

from azimuthCalibration import turnAZ, turnEL
import sys
import pigpio

angleAZ = float(sys.argv[1])
angleEL = float(sys.argv[2])

pi = pigpio.pi()

turnAZ(0, pig = pi)
turnAZ(angleAZ, pig = pi) # writing to serial
#turnEL(0, pig = pi)
#turnEL(angleEL, pig = pi)
