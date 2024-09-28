#!/usr/bin/python
#
#       This is the base code needed to get usable angles from a BerryIMU
#       using a Complementary filter. The readings can be improved by
#       adding more filters, E.g Kalman, Low pass, median filter, etc..
#       See berryIMU.py for more advanced code.
#
#       The BerryIMUv1, BerryIMUv2 and BerryIMUv3 are supported
#       This script is python 2.7 and 3 compatible
#
#       Feel free to do whatever you like with this code.
#       Distributed as-is; no warranty is given.
#
#       https://ozzmaker.com/berryimu/


import time
import math
import IMU
import datetime
import os
import sys
import serial

def getHeading(magMin = [-2913, -376, -2809], magMax = [611, 3008, 817], msg = False, nIter = 5):
    """Gets the heading angle for the compass.

        params
        ------
        msg : boolean
            If true then print out info on acceleration, gyro, CF angle, heading. If false do not print these items.
    """

    IMU.detectIMU()     #Detect if BerryIMU is connected.
    if(IMU.BerryIMUversion == 99):
    	print(" No BerryIMU found... exiting ")
    	sys.exit()
    IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

    RAD_TO_DEG = 57.29578
    M_PI = 3.14159265358979323846
    G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
    AA =  0.40      # Complementary filter constant

    ################# Compass Calibration values ############
    # Use calibrateBerryIMU.py to get calibration values
    # Calibrating the compass isnt mandatory, however a calibrated
    # compass will result in a more accurate heading values.

    magXmin = magMin[0]
    magYmin = magMin[1]
    magZmin = magMin[2]
    magXmax = magMax[0]
    magYmax = magMax[1]
    magZmax = magMax[2] 



    '''
    Here is an example:

    magXmin =  -1748
    magYmin =  -1025
    magZmin =  -1876
    magXmax =  959
    magYmax =  1651
    magZmax =  708
    Dont use the above values, these are just an example.
    '''
    ############### END Calibration offsets #################



    gyroXangle = 0.0
    gyroYangle = 0.0
    gyroZangle = 0.0
    CFangleX = 0.0
    CFangleY = 0.0

    a = datetime.datetime.now()

    for i in range(nIter):
    
        #Read the accelerometer,gyroscope and magnetometer values
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()
    
        #Apply compass calibration
        MAGx -= (magXmin + magXmax) /2
        MAGy -= (magYmin + magYmax) /2
        MAGz -= (magZmin + magZmax) /2
    
        ##Calculate loop Period(LP). How long between Gyro Reads
        b = datetime.datetime.now() - a
        a = datetime.datetime.now()
        LP = b.microseconds/(1000000*1.0)
        outputString = "Loop Time %5.2f " % ( LP )
    
    
        #Convert Gyro raw to degrees per second
        rate_gyr_x =  GYRx * G_GAIN
        rate_gyr_y =  GYRy * G_GAIN
        rate_gyr_z =  GYRz * G_GAIN
    
    
        #Calculate the angles from the gyro.
        gyroXangle+=rate_gyr_x*LP
        gyroYangle+=rate_gyr_y*LP
        gyroZangle+=rate_gyr_z*LP
    
    
        #Convert Accelerometer values to degrees
        AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
        AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
    
        #convert the values to -180 and +180
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0
    
    
    
        #Complementary filter used to combine the accelerometer and gyro values.
        CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
        CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle
    
    
    
        #Calculate heading
        heading = 180 * math.atan2(MAGy,MAGx)/M_PI
    
        #Only have our heading between 0 and 360
        if heading < 0:
            heading += 360
    
        ####################################################################
        ###################Tilt compensated heading#########################
        ####################################################################
        #Normalize accelerometer raw values.
        accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    
    
        #Calculate pitch and roll
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm/math.cos(pitch))
    
    
        #Calculate the new tilt compensated values
        #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
        #This needs to be taken into consideration when performing the calculations
    
        #X compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        else:                                                                #LSM9DS1
            magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)
    
        #Y compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
        else:                                                                #LSM9DS1
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)
    
    
    
    
        #Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
    
        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360
    
    
        ##################### END Tilt Compensation ########################
    
    
        if msg:                       #Change to '0' to stop showing the angles from the accelerometer
            outputString += "#  ACCX Angle %5.2f ACCY Angle %5.2f  #  " % (AccXangle, AccYangle)
    
        if msg:                       #Change to '0' to stop  showing the angles from the gyro
            outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)
    
        if msg:                       #Change to '0' to stop  showing the angles from the complementary filter
            outputString +="\t#  CFangleX Angle %5.2f   CFangleY Angle %5.2f  #" % (CFangleX,CFangleY)
    
        if msg:                       #Change to '0' to stop  showing the heading
            outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)
    
    
        print(outputString)
    
    return (heading, tiltCompensatedHeading, (AccXangle, AccYangle), (gyroXangle, gyroYangle, gyroZangle), (CFangleX, CFangleY))

def calibrateMag(angleIncrement = 45):
    IMU.detectIMU()
    IMU.initIMU()

    ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
    ser.reset_input_buffer()    

    a = datetime.datetime.now()


    #Preload the variables used to keep track of the minimum and maximum values
    magXmin = 32767
    magYmin = 32767
    magZmin = 32767
    magXmax = -32767
    magYmax = -32767
    magZmax = -32767

    turnAZ(ser, 0)

    for i in range(0, 180):
        if (i + 1) % angleIncrement == 0:
            print("({},{},{}), ({},{},{})".format(\
                magXmin, magYmin, magZmin, magXmax, magYmax, magZmax))
            turnAZ(ser, i + 1)
    

        a = datetime.datetime.now()
        #Read magnetometer values
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        if MAGx > magXmax:
            magXmax = MAGx
        if MAGy > magYmax:
            magYmax = MAGy
        if MAGz > magZmax:
            magZmax = MAGz

        if MAGx < magXmin:
            magXmin = MAGx
        if MAGy < magYmin:
            magYmin = MAGy
        if MAGz < magZmin:
            magZmin = MAGz

        for j in range(0, 91):
            if j % angleIncrement == 0:
                print("({},{},{}), ({},{},{})".format(\
                    magXmin, magYmin, magZmin, magXmax, magYmax, magZmax))
                turnEL(ser, j)

            if MAGx > magXmax:
                magXmax = MAGx
            if MAGy > magYmax:
                magYmax = MAGy
            if MAGz > magZmax:
                magZmax = MAGz

            if MAGx < magXmin:
                magXmin = MAGx
            if MAGy < magYmin:
                magYmin = MAGy
            if MAGz < magZmin:
                magZmin = MAGz



    turnAZ(ser, 0)

    for i in range(-180, 0):
        if i % angleIncrement == 0:
            print("({},{},{}), ({},{},{})".format(\
                magXmin, magYmin, magZmin, magXmax, magYmax, magZmax))
            turnAZ(ser, 360 + i)


        a = datetime.datetime.now()
        #Read magnetometer values
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        if MAGx > magXmax:
            magXmax = MAGx
        if MAGy > magYmax:
            magYmax = MAGy
        if MAGz > magZmax:
            magZmax = MAGz

        if MAGx < magXmin:
            magXmin = MAGx
        if MAGy < magYmin:
            magYmin = MAGy
        if MAGz < magZmin:
            magZmin = MAGz

        for j in range(0, 91):
            if j % angleIncrement == 0:
                print("({},{},{}), ({},{},{})".format(\
                    magXmin, magYmin, magZmin, magXmax, magYmax, magZmax))
                turnEL(ser, j)

            if MAGx > magXmax:
                magXmax = MAGx
            if MAGy > magYmax:
                magYmax = MAGy
            if MAGz > magZmax:
                magZmax = MAGz

            if MAGx < magXmin:
                magXmin = MAGx
            if MAGy < magYmin:
                magYmin = MAGy
            if MAGz < magZmin:
                magZmin = MAGz


    return ([magXmin, magYmin, magZmin],[magXmax, magYmax, magZmax])

#TODO: swap delay for a check of serial command TURNDONE            
def turnAZ(ser, deg, sleepTime = 5):
    """Pass a serial object and desired number of degrees to turn the azimuth motor.
    The angle must be positive.
    """
    ser.write("AZ{}".format(float(deg)).encode())
    time.sleep(sleepTime)
    
def turnEL(ser, deg, sleepTime = 5):
    """Pass a serial object and desired number of degrees to turn the elevation motor.
    The angle must be positive.
    """
    ser.write("EL{}".format(float(deg)).encode())
    time.sleep(sleepTime)

def sendZeroAZ(ser, deg, sleepTime = 5):
    """Pass a serial object and the angle at which the azimuth is zeroed (points north).
    """
    ser.write("ZO{}".format(float(deg)).encode())
    time.sleep(sleepTime)
    
