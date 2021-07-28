#!/usr/bin/env python

import sys
import moveit_commander
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import os
import numpy as np
import math
import tf
import tf2_ros
import geometry_msgs.msg


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion_planning1', anonymous=True)
tfBuffer = tf2_ros.Buffer() 				
listener = tf2_ros.TransformListener(tfBuffer)		
rate = rospy.Rate(10.0)					

scene = PlanningSceneInterface()
robot = RobotCommander()
arm_l = MoveGroupCommander("both_arms")
l = MoveGroupCommander("arm_l")
r = MoveGroupCommander("arm_r")
	
while(True):
		     

	display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
	end_effector_link1 = l.get_end_effector_link()
	end_effector_link = r.get_end_effector_link()
	print(end_effector_link)
	reference_frame = 'base_link'
	try:
		print("check before trans")                
		trans = tfBuffer.lookup_transform('camera_rgb_optical_frame', 'object_29', rospy.Time())
		print("check passed trans")
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
	    	print("check unpassed trans")                
	    	rate.sleep()
            	continue
	arm_l.allow_replanning(True)

	target_pose = PoseStamped()
	target_pose1 = PoseStamped()
	home_l = PoseStamped()
	home_r = PoseStamped()
	arm_l.set_pose_reference_frame(reference_frame)

	rospy.sleep(2)
	 
	arm_l.set_num_planning_attempts(10000);

		
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
	x= BAr_Z #+ 0.2
	y1= -BAr_X-0.4
	y=  -BAr_X+0.4
	z= 1.12-BAr_Y

	arm_l.go()
	arm_l.set_position_target( [x, y1, z] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
	arm_l.set_position_target( [x, y, z] ,end_effector_link)

	arm_l.set_goal_position_tolerance(0.0001)
	arm_l.go(wait=True)

	#arm_l.set_rpy_target(  [-1.252, 0.319, 1.006] ,end_effector_link1)
	#arm_l.set_rpy_target(  [-1.183, 0.405, 0.980] ,end_effector_link)
	arm_l.set_goal_orientation_tolerance(0.0001)

	arm_l.go(wait=True)
	print l.get_current_pose()
	print r.get_current_pose()
	rospy.sleep(5)
	break
