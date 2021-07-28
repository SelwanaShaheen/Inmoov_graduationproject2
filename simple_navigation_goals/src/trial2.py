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
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander

pub= rospy.Publisher('Goal2', Int16 , queue_size=10)
rospy.init_node('motion_planning1', anonymous=True)

def callback1(data):
	global sub
	sub=data.data
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", sub)
	if sub == 10:
	    print("DONE")
	    rospy.signal_shutdown("5ara")


while(True):
	try:
		rospy.Subscriber("Goal1", Int16, callback1)
		rospy.spin() 
		print ("pub1")
		print("HOME_P")
		rospy.sleep(5)            
		print("PREPARE_P")
		rospy.sleep(5)            
		print("CATCH_P")
		rospy.sleep(5)            
		print("LIFT_P")
		rospy.sleep(5)            
		pub.publish(20)
		print("MOVEBASE_P")
		if result:              
			rospy.loginfo("Goal 2 Reached!!!")
			print("PLACE_P") 
			rospy.sleep(5)            
			print("RELEASE_P") 
			rospy.sleep(5)
			break

	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
