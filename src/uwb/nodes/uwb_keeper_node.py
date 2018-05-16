#!/usr/bin/env python

# Copyright (c) 2018, Tobias K.
# All rights reserved.


import rospy
import serial
import string
import math
import sys
import time
import csv

from range_msg.msg import uwb

rospy.init_node("uwb-node")
pub = rospy.Publisher('/uwb', uwb, queue_size=1)
diag_pub_time = rospy.get_time();

uwbMsg = uwb()

default_port='/dev/ttyUSB0'
port = default_port

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("UWB not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

rospy.loginfo("Giving the UWB board time to boot...")
rospy.sleep(1) # Sleep for 1 seconds to wait for the board to boot

rospy.loginfo("UWB Node booted")
rospy.sleep(1)

rospy.loginfo("Enable Debug Mode")
if ser.readline() == "":
    ser.write('\xF0\x1C')
rospy.sleep(1)

#log uwb data to usb-stick
lognamekeeper = "/media/pi/disk/logdata/uwb/Keeper/uwb_keeper_"
lognamekeeper += time.strftime("%Y%m%d-%H%M%S")
lognamekeeper += ".csv"

lognameksmall = "/media/pi/disk/logdata/uwb/Ksmall/uwb_ksmall_"
lognameksmall += time.strftime("%Y%m%d-%H%M%S")
lognameksmall += ".csv"

with open (lognamekeeper, 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Timestamp", "Antenna_Number", "EuiD", "Distance"])

with open (lognameksmall, 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Timestamp", "EuiD", "Distance"])

rospy.loginfo("Publishing UWB Data")

#pos_of_euid = 0
#antennas = []
#euids = []
#distances = []


rospy.loginfo("First Data flushed")
rospy.sleep(1)
discard = ser.readline()
discard = ser.readline()
discard = ser.readline()

indexErr = 0
valueErr = 0
formatErr = 0
antennaErr = 0

while not rospy.is_shutdown():
    
    line = ser.readline()
    
    if indexErr == 50:
        indexErr = 0
        rospy.logwarn("50 Index Errors counted in Uwb ..... Counter resetted")
    
    if valueErr == 50:
        valueErr = 0
        rospy.logwarn("50 Value Errors counted in Uwb ..... Counter resetted")
    
    if formatErr == 50:
        formatErr = 0
        rospy.logwarn("50 Format Errors counted in Uwb ..... Counter resetted")
        
    if antennaErr == 50:
        antennaErr = 0
        rospy.logwarn("50 Antenna Errors counted in Uwb ..... Counter resetted")
    
    if not line == "":
        words = string.split(line,";")
        
        try:
            numbers = string.split(words[0],",")
        except IndexError:
            #rospy.logwarn("Wrong Index in words[0]")
            
            continue
        try:
            antenna = numbers[0]
        except IndexError:
            #rospy.logwarn("Wrong Index in numbers[0]")
            indexErr = indexErr + 1
            continue
        try:
            euid = numbers[1]
        except IndexError:
            #rospy.logwarn("Wrong Index in numbers[1]")
            indexErr = indexErr + 1
            continue
        
        euid = euid.replace(" ","")
        eiud = euid.replace("\x00","")
        #if len(euid) > 16 and len(euid) < 16:
         #   rospy.logwarn("EuiD has wrong Format")
          #  continue
        
        try:
            try:
                dist = float(words[1])/100
            except ValueError:
                #rospy.logwarn("Value Error for dist")
                valueErr = valueErr + 1
                dist = 0.0
                continue
        except IndexError:
            #rospy.logwarn("Wrong Index in words[1]")
            indexErr = indexErr + 1
            continue
        
        #if not euid in euids:
        #    euids.append(euid)
        #    distances.append(dist)
        #else:
        #    ids = euids
        #    for i, ids in enumerate(ids):
        #        if(euid == ids):
        #            pos_of_euid = i
        #    distances[pos_of_euid] = dist
    
        #if len(euids) > 0:
        #    rospy.loginfo(antenna)
        #    rospy.loginfo(euids)
        #    rospy.loginfo(distances)
    
        if not len(euid) == 16: 
            #rospy.logwarn("EuiD has wrong Format")
            formatErr = formatErr + 1
            continue
        if not (antenna == "1" or antenna == "2" or antenna == "3" or antenna =="4") or antenna == "":
            #rospy.logwarn("Wrong Antenna Number")
            antennaErr = antennaErr + 1
            continue
        
        if antenna == "4":
            uwbMsg.header.stamp= rospy.Time.now()
            uwbMsg.euid = euid
            uwbMsg.distance = dist
            datalist = [uwbMsg.header.stamp, euid, dist]
            with open (lognameksmall, 'a') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(datalist)
      
            pub.publish(uwbMsg)
        
        else:
            datalist = [uwbMsg.header.stamp, antenna, euid, dist]
            with open (lognamekeeper, 'a') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(datalist)
        
    else:
        rospy.sleep(1)
        rospy.loginfo("No Data received")

ser.write('\xF0\x1C')
ser.close
#f.close
