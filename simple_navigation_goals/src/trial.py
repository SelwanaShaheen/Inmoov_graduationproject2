#!/usr/bin/env python  
import rospy
import os
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def move_base(bar_x, bar_y, bar_z,bar_rx, bar_ry, bar_rz, bar_rw):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = bar_x-0.5
    goal.target_pose.pose.position.y = bar_y-0.1
    goal.target_pose.pose.position.z = 0
    #goal.target_pose.pose.orientation.x = bar_rz
    goal.target_pose.pose.orientation.y = bar_ry
    goal.target_pose.pose.orientation.z = bar_rw #bar_rz
    goal.target_pose.pose.orientation.w = -bar_rz #bar_rw
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('ar_tag_navigation')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
	x=1
	i=0
	a = np.array(['object_17','object_18','object_19','object_20','object_21','object_22','object_23'])
        #while not rospy.is_shutdown():
	while(x==1):
	
            try:
                #trans = tfBuffer.lookup_transform('camera_rgb_optical_frame', 'object_8', rospy.Time())
		#trans = tfBuffer.lookup_transform('kinect2_rgb_optical_frame', 'object_30', rospy.Time())
		trans = tfBuffer.lookup_transform('camera_link', a[i], rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
               # rate.sleep()
		if i==(len(a)-1):
			i=0
	    	else:
			i=i+1
			
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
	    if (BAr_X !=0) or (BAr_Y !=0) or (BAr_Z !=0):
		print(a[i])
		print("before")
		os.system("rosnode kill "+ '/move_base_sequence')
                #os.system("rosnode kill "+ '/explore')
		print("after")
		x = 0
		#if BAr_rW < -0.01:
		#	BAr_Y=BAr_Y+0.2
	#		result = move_base(BAr_X, BAr_Y,BAr_Z, BAr_rX, BAr_rY, BAr_rZ, BAr_rW)
	#	elif BAr_rW >-0.01:
	#		BAr_Y=BAr_Y-0.1
         #   		result = move_base(BAr_X, BAr_Y,BAr_Z, BAr_rX, BAr_rY, BAr_rZ, BAr_rW)
	#	else:
       		result = move_base(BAr_X, BAr_Y,BAr_Z, BAr_rX, BAr_rY, BAr_rZ, BAr_rW)
		print("result")
	    	#x = 0
            if result:
                rospy.loginfo("Goal execution done!")
		break		


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
