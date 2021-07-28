#!/usr/bin/env python
import rospy
import freenect
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np 
from numpy import float32
import math
from geometry_msgs.msg import Pose
import time
import sys
import struct
from tf import transformations
from sensor_msgs.msg import PointCloud2, PointField, Image
from ros_numpy import numpy_msg

def image_callback(img_msg):
    print ("ana image") 
    cv_image=bridge.imgmsg_to_cv2(img_msg,'bgr8')
    #cv_image = ros_numpy.numpify(Camera, img_msg)
    cv2.imshow(" Image", cv_image)

def depth_callback(img_msg):
    global depth_imag
    depth_imag = bridge.imgmsg_to_cv2(img_msg,"32FC1")
    print ("ana depth")
    print (depth_imag)
    cv2.imshow("Depth Image", depth_imag) 

def get_depth(y,x):

    cx =314.0137
    cy = 247.90585
    fx = 591.1027
    fy =590.557
 
    w = 640
    h = 480

    array,_ = freenect.sync_get_depth()
    array = array.astype(np.float32)
    #index = x + y*w
    depth_value = array[y,x]
    distance_cm = 100/(-0.00307 * depth_value + 3.33)


    depthInMeters = 1.0 / (depth_value * -0.0030711016 + 3.3309495161)   

       
    

    z= depthInMeters
    x = ( x - cx)*z/fx
    y = (y - cy)*z/fy  
  
    #x = (2 * math.tan(29 * 3.14159265359 / 180) * z) * ((x - w/2) / 640)
    #y = (2 * math.tan(22.5 * 3.14 / 180) * z) * ((y - h/2) / 480)
    real_values = [z,-x,-y]
    #cv2.imshow('depth', array)
    print(z,x,y)

def getDepthMap():  
    depth, timestamp = freenect.sync_get_depth()
    #print ("depth el function", depth)
    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.float32)

    return depth

rospy.Subscriber("/camera/rgb/image_color",Image,image_callback)
rospy.Subscriber("/camera/depth/image_raw",Image,depth_callback

body_classifier = cv2.CascadeClassifier('/home/miriam/control/src/yarab_5/script/whiteboxdetector.xml')

while True:

    rospy.init_node('detection',anonymous=True)
    bridge=CvBridge()



    depthmap = getDepthMap()
    
   
    blur = cv2.resize(depthmap , None , fx=0.7, fy =0.7 )
    cv2.waitKey(1)


    #print (depth_imag)
    frame = freenect.sync_get_video()[0]
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = cv2.resize(frame, None,fx=0.7, fy=0.7 ,interpolation = cv2.INTER_LINEAR)
    #if (type(frame) == type(None)):
     #   break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Pass frame to our body classifier
    bodies = body_classifier.detectMultiScale(gray, minSize =(20, 20))
    
    # Extract bounding boxes for any bodies identified
    amount_found = len(bodies) 
    if amount_found != 0: 
        for (x,y,w,h) in bodies:
            cv2.rectangle(blur, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
            get_depth(y,x)
    cv2.imshow('image', blur)    
    cv2.imshow('Object', frame)


    if cv2.waitKey(1) == 13: #13 is the Enter Key
        break
