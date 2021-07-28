#!/usr/bin/env python  
import rospy
import os
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
import actionlib


rospy.init_node('kinect')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10.0)
x=1
while(x==1):
	try:
   		trans = tfBuffer.lookup_transform('camera_rgb_optical_frame', 'object_20', rospy.Time())
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
   		rate.sleep()
   		continue

	BAr_X = trans.transform.translation.x
	print(BAr_X)
	BAr_Y = trans.transform.translation.y
	print(BAr_Y)
	BAr_Z = trans.transform.translation.z
	print(BAr_Z)
	BAr_rX = trans.transform.rotation.x
	print(BAr_rX)
	BAr_rY = trans.transform.rotation.y
	print(BAr_rY)
	BAr_rZ = trans.transform.rotation.z
	print(BAr_rZ)
	BAr_rW = trans.transform.rotation.w
	print(BAr_rW)
