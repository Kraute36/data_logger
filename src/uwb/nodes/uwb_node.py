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
logname = "/media/pi/disk1/logdata/uwb/uwb_"
logname += time.strftime("%Y%m%d-%H%M%S")
logname += ".csv"

with open (logname, 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Timestamp", "EuiD", "Distance"])

rospy.loginfo("Publishing UWB Data")

pos_of_euid = 0
euids = []
distances = []

discard = ser.readline()
rospy.loginfo("first data flushed")


while not rospy.is_shutdown():
     
    line = ser.readline() 
    words = string.split(line,";")
    euid = words[0]
    dist = float(words[1])/100
    
    if not euid in euids:
        euids.append(euid)
        distances.append(dist)
    else:
        ids = euids
        for i, ids in enumerate(ids):
            if(euid == ids):
                pos_of_euid = i
        distances[pos_of_euid] = dist
    
    if len(euids) > 2:
        rospy.loginfo(euids)
        rospy.loginfo(distances)
    
    uwbMsg.header.stamp= rospy.Time.now()
    uwbMsg.euid = euid
    uwbMsg.distance = dist
    datalist = [uwbMsg.header.stamp, euid, dist]
    with open (logname, 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(datalist)
      
    pub.publish(uwbMsg)

ser.write('\xF0\x1C')
ser.close
#f.close
