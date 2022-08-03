    
#define function to calculate cross product 
def cross_prod(a, b):
    result = [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]

    return result





#!/usr/bin/python
#
#       This is the base code needed to get usable angles from a BerryIMU
#       using a Complementary filter. The readings can be improved by
#       adding more filters, E.g Kalman, Low pass, median filter, etc..
#       See berryIMU.py for more advanced code.
#
#       The BerryIMUv1, BerryIMUv2 and BerryIMUv3 are supported
#
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
import csv

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant

################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values
# Calibrating the compass isnt mandatory, however a calibrated
# compass will result in a more accurate heading values.

magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0



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


IMU.detectIMU()     #Detect if BerryIMU is connected.
if(IMU.BerryIMUversion == 99):
    print(" No BerryIMU found... exiting ")
    sys.exit()
IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass


a = datetime.datetime.now()

q = {}  #qauternion
lix = []
liy = []
liz = []
yawInit = 0
count = 0

VELx = 0
VELy = 0

POSx = 0
POSy = 0

newDict = []
dt = 0.05

start1 = time.time()

while (True):
    duration = time.time()-start1
    print(time.time()-start1)

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



    #Calculate heading                         Change in Heading is yaw
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




    #Calculate tilt compensated heading/yaw
    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

    if tiltCompensatedHeading < 0:
        tiltCompensatedHeading += 360
    
    
    yG = ((ACCy * 0.244)/1000)
    xG = ((ACCx * 0.244)/1000)
    zG = ((ACCz * 0.244)/1000)
    
    
    count+=1

    if (count==1):
        yawInit = tiltCompensatedHeading
        yaw = 0
    else:
        yaw = tiltCompensatedHeading-yawInit
        
    if (yaw<0):
        yaw+=360 
    #print(yaw)
    
    if (yaw < 90):
        xACCe = xG*math.cos(yaw*M_PI/180)+yG*math.sin(yaw*M_PI/180)
        yACCe = xG*math.sin(yaw*M_PI/180)-yG*math.cos(yaw*M_PI/180)
    if (yaw >=90 and yaw < 180):
        xACCe = -1*xG*math.sin((yaw-90)*M_PI/180)+yG*math.cos((yaw-90)*M_PI/180)
        yACCe = -1*xG*math.cos((yaw-90)*M_PI/180)-yG*math.sin((yaw-90)*M_PI/180)
    if (yaw>=180 and yaw <270):
        xACCe = -1*xG*math.cos((yaw-180)*M_PI/180)-yG*math.sin((yaw-180)*M_PI/180)
        yACCe = xG*math.sin((yaw-180)*M_PI/180)-yG*math.cos((yaw-180)*M_PI/180)
    if (yaw>=270):
        XACCe = xG*math.cos((360-yaw)*M_PI/180)-yG*math.sin((360-yaw)*M_PI/180)
        yACCe = xG*math.sin((360-yaw)*M_PI/180)+yG*math.cos((360-yaw)*M_PI/180)
    
    #print("##### accXe = %fG  ##### accYe =   %fG" % ( xACCe, yACCe))
    RAWaccx = xACCe
    RAWaccy = yACCe

    alpha = 0.6
    accIgTh = 0.04             #acc ignore thresh

    if (count == 1):
        LPx = 0                                     #assuming starting from rest and paralell to ground (if initial condition not met then use gyro and qauternion to rotate g vector)
        LPy = 0
        LPz = 1
        LPxOld = LPx
        LPyOld = LPy
    else:
        LPxOld = round(LPx,2)
        LPyOld = round(LPy,2)


        LPx = (LPx*(1-alpha)) + (xACCe * alpha)
        LPy = (LPy*(1-alpha)) + (yACCe * alpha)
        

    xACCe = LPx
    yACCe = LPy

    xACCe = round(xACCe, 2)
    yACCe = round(yACCe, 2)

    if (abs(xACCe-LPxOld)<=accIgTh):
        xACCe = LPxOld

    if (abs(yACCe-LPyOld)<=accIgTh):
        yACCe = LPyOld



    if ((abs(xACCe)<=accIgTh)):
        xACCe = 0

    if ((abs(yACCe)<=accIgTh)):
        yACCe = 0

   

    xACCe = round(xACCe, 2)
    yACCe = round(yACCe, 2)
#    print("##### accXe = %fG  ##### accYe =   %fG" % ( xACCe, yACCe))

    
    VELx += xACCe*9.8*LP
    VELy += yACCe*9.8*LP
 #   print("##### VELx = %f  ##### VELy =   %f" % ( VELx, VELy))

    if (xACCe == 0 and yACCe == 0):
        VELx = 0
        VELy = 0

    POSx += VELx*LP
    POSy += VELy*LP



    
    ##################### END Tilt Compensation ########################


    if 1:                       #Change to '0' to stop showing the angles from the accelerometer
        outputString += "#  ACCX Angle %5.2f ACCY Angle %5.2f  #  " % (AccXangle, AccYangle)

    if 1:                       #Change to '0' to stop  showing the angles from the gyro
        outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)

    if 1:                       #Change to '0' to stop  showing the angles from the complementary filter
        outputString +="\t#  CFangleX Angle %5.2f   CFangleY Angle %5.2f  #" % (CFangleX,CFangleY)

    if 1:                       #Change to '0' to stop  showing the heading
        outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)


    #print(outputString)
    #print(pitch)


    #slow program down a bit, makes the output more readable
    time.sleep(dt)
    
    csvdata = {"Time": duration,"RAW ACCX": RAWaccx, "RAW ACCY": RAWaccy, "ACCX AFTER LP": LPx, "ACCY AFTER LP":LPy ,"ACCXe": xACCe, "ACCYe":yACCe,"VELx":VELx, "VELy":VELy,"x":POSx, "y":POSy}
    newDict.append(csvdata)
    print(csvdata)


#fieldnames = ["Time", "RAW ACCX", "RAW ACCY","ACCX AFTER LP","ACCY AFTER LP","ACCXe", "ACCYe", "VELx", "VELy", "x", "y"]
#with open('data.csv', 'w', encoding='UTF8', newline='') as f:
#    writer = csv.DictWriter(f, fieldnames=fieldnames)
#    writer.writeheader()
#    writer.writerows(newDict)

