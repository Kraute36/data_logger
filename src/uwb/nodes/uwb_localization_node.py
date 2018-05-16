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
import numpy as np
import random
import tf

from range_msg.msg import uwb
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

#def trilaterate2d(id, range):
def trilaterate2d(distance):    
    p1 = np.array([0, 4.8])
    p2 = np.array([5.1, 0])
    p3 = np.array([5.1, 9.2])
    #p4 = np.array([10, 0, 0])
    #values = [-1, 1]
    #r1 = 10*np.sqrt(2)/2 + random.choice(values) * (10*np.sqrt(2)/2)/random.randint(5,100)
    #r2 = 10*np.sqrt(2)/2 + random.choice(values) * (10*np.sqrt(2)/2)/random.randint(8,100)
    #r3 = 10*np.sqrt(2)/2 + random.choice(values) * (10*np.sqrt(2)/2)/random.randint(7,100)
    #r4 = 10*np.sqrt(2)/2 + random.choice(values) * (10*np.sqrt(2)/2)/random.randint(10,100)
    r1 = distance[0]
    r2 = distance[1]
    r3 = distance[2]
    e_x = (p2-p1)/np.linalg.norm(p2-p1)
    i = np.dot(e_x, (p3-p1))
    e_y=(p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    e_z=np.cross(e_x,e_y)
    d=np.linalg.norm(p2-p1)
    j=np.dot(e_y,(p3-p1))
    x=((r1**2)-(r2**2)+(d**2))/(2*d)
    y=(((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    #z1=np.sqrt(r1**2-x**2-y**2)
    #z2=np.sqrt(r1**2-x**2-y**2)*(-1)
    ans1=p1+(x*e_x)+(y*e_y)#+(z1*e_z)
    #ans2=p1+(x*e_x)+(y*e_y)+(z2*e_z)
    #dist1=np.linalg.norm(p4-ans1)
    #dist2=np.linalg.norm(p4-ans2)
    return ans1
    if np.abs(r4-dist1)<np.abs(r4-dist2):
        return ans1
    else: 
        return ans2
    


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
logname = "/media/pi/disk/logdata/uwb/uwb_"
logname += time.strftime("%Y%m%d-%H%M%S")
logname += ".csv"

#with open (logname, 'a') as csvfile:
#        writer = csv.writer(csvfile)
#        writer.writerow(["Timestamp", "EuiD", "Distance"])

rospy.loginfo("Publishing UWB Data")

euids =['100022B410206000','10001EDC10206000','100023CA10206000']
distances = [0.0,0.0,0.0]
pos_of_euid = 0
pos = np.array([0,0])
oldpos = pos

discard = ser.readline()
rospy.loginfo("first data flushed")

#odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
#odom = Odometry()
#br = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    #pos = 0.1 * trilaterate2d() + 0.9 * oldpos
    #rpos = np.array([round(pos[0],2),round(pos[1],2)])
    #rospy.loginfo(rpos)
    #oldpos = pos
    
    #odom.header.stamp = rospy.Time.now()
    #odom.header.frame_id = "odom"
    #odom.pose.pose = Pose(Point(round(pos[0],2), round(pos[1],2), 0), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0)))
    #odom_pub.publish(odom)
    #rospy.sleep(0.2)
    #br.sendTransform((round(pos[0],2), round(pos[1],2),0), tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"tfbroad","world")
    
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
        #rospy.loginfo(euids)
        #rospy.loginfo(distances)
        pos = 0.4 * trilaterate2d(distances) + 0.6 * oldpos
        rpos = np.array([round(pos[0],2),round(pos[1],2)])
        rospy.loginfo(rpos)
    #    rospy.sleep(0.25)
        oldpos = pos
    
    uwbMsg.header.stamp= rospy.Time.now()
    uwbMsg.euid = euid
    uwbMsg.distance = dist
 #   datalist = [uwbMsg.header.stamp, euid, dist]
 #   with open (logname, 'a') as csvfile:
 #       writer = csv.writer(csvfile)
 #       writer.writerow(datalist)
      
    pub.publish(uwbMsg)

ser.write('\xF0\x1C')
ser.close
#f.close
