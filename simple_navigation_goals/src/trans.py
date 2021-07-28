#!/usr/bin/env python  
import rospy
import os
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def move_base(bar_x, bar_y, bar_z,roll, pitch, yaw,bar_rz,bar_rw):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = bar_x #bar_z-0.5
    goal.target_pose.pose.position.y = bar_y#-bar_x
    goal.target_pose.pose.position.z = bar_z #0
    #goal.target_pose.pose.orientation.x = bar_rz #bar_rz
    goal.target_pose.pose.orientation.y = bar_ry#bar_rz
    goal.target_pose.pose.orientation.z = bar_rz #bar_rz*0.009#bar_rz #bar_rz
    goal.target_pose.pose.orientation.w = bar_rw #bar_rw*-0.01 #bar_rw-3 #bar_rw
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
	a = np.array(['object_29', 'object_30', 'object_31', 'object_32'])
        #while not rospy.is_shutdown():
	while(x==1):
	
            try:
                #trans = tfBuffer.lookup_transform('camera_rgb_optical_frame', 'object_8', rospy.Time())
		#trans = tfBuffer.lookup_transform('kinect2_rgb_optical_frame', 'object_30', rospy.Time())
		trans = tfBuffer.lookup_transform('base_link', a[i], rospy.Time())
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
	    orientation_list = [BAr_rX, BAr_rY, BAr_rZ, BAr_rW]
 	    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	    if (BAr_X !=0) or (BAr_Y !=0) or (BAr_Z !=0):
		print(a[i])
		print("before")
		os.system("rosnode kill "+ '/explore')
		print("after")
		x = 0
            	result = move_base(BAr_X, BAr_Y,BAr_Z, roll, pitch, yaw,BAr_rZ,BAr_rW)
		print("result")
	    	#x = 0
            if result:
                rospy.loginfo("Goal execution done!")
		break		


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
