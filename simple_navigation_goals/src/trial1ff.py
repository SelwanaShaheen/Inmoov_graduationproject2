#!/usr/bin/env python  
import rospy
import os
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
import actionlib
from std_msgs.msg import String, Int16
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global pub
global pub1

def move_base(bar_x,bar_y,bar_z,bar_rx, bar_ry, bar_rz, bar_rw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x =bar_x-0.4 #x#bar_x-0.5
    goal.target_pose.pose.position.y = bar_y-0.1#y#bar_y+0.1
    goal.target_pose.pose.position.z = bar_z# z#bar_z
    #goal.target_pose.pose.orientation.x = bar_rz
    goal.target_pose.pose.orientation.y = bar_ry
    goal.target_pose.pose.orientation.z = bar_rw #bar_rz
    goal.target_pose.pose.orientation.w = -bar_rz
    client.send_goal(goal)
    wait = client.wait_for_result()
    #if not wait:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")
    #else:
    return client.get_result()

#def callback2(data):
#		global sub1
#		sub1=data.data
#		rospy.loginfo(rospy.get_caller_id() + "I heard %s", sub1)
#		if sub1 == 2:
#			print("enter1")
#			result1 = move_base(1,0,0,1)
			#result1 = 5	
#			rospy.sleep(2)	
#			print("enter2")
#			if result1:            
#				pub1.publish(3)
#				print("Goal 2 reached")			
#				print (result1)
#				rospy.signal_shutdown("5ara")


if __name__ == '__main__':
	try:
		rospy.init_node('ar_tag_navigation')
		pub= rospy.Publisher('Goal1_movebase', Int16 , queue_size=10)
		#pub1= rospy.Publisher('Goal2_movebase', Int16 , queue_size=10)
		rospy.sleep(5)
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		a = np.array(['object_1'])
		rate = rospy.Rate(10.0)

		#x = 1
		#x1=1
		i=0
		while(True):
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
				print("after")
				#x = 0
				result = move_base(BAr_X, BAr_Y,BAr_Z, BAr_rX, BAr_rY, BAr_rZ, BAr_rW)
				print("result")
					#x = 0
				if result:
					pub.publish(1)
					print("Goal1 reached")
					print (result)
					break
		#print("wait for inverse")
		#rospy.Subscriber("picking", Int16, callback2)
		#rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
