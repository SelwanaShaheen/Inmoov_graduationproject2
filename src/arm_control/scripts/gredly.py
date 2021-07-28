#!/usr/bin/env python

import sys
import moveit_commander
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import rospy
import os
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion_planning1', anonymous=True)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10.0)
scene = PlanningSceneInterface()
robot = RobotCommander()
arm_l = MoveGroupCommander("arm_l")

while not rospy.is_shutdown():
	try:
   		trans = tfBuffer.lookup_transform('camera_rgb_optical_frame', 'object_21', rospy.Time())
		print("check1")
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
   		rate.sleep()
   		continue
	print("check2")
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



	display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
	 
	end_effector_link = arm_l.get_end_effector_link()
	reference_frame = 'base_link'
	arm_l.set_pose_reference_frame(reference_frame)
	arm_l.allow_replanning(True)

	#print(arm_l.get_current_pose())
	arm_l.set_goal_position_tolerance(0.5)
	arm_l.set_goal_orientation_tolerance(0.000005);

	target_pose = PoseStamped()#Pose()
	target_pose.header.frame_id = reference_frame
	target_pose.header.stamp = rospy.Time.now()
	target_pose.pose.position.x =  1.32500
	target_pose.pose.position.y = -0.952100
	target_pose.pose.position.z = -1.43500
	#target_pose.pose.orientation.x = 0.0605532236567
	#target_pose.pose.orientation.y = -0.350735920434
	#target_pose.pose.orientation.z = 0.152864566593
	target_pose.pose.orientation.w = 1.879632033407
	#arm_l.set_position_target(target_pose)


	#arm_l.set_named_target('goal2')
	#arm_l.go()
	rospy.sleep(2)
	    
	print(arm_l.get_current_pose())           
	#print(l.get_current_pose())
	#print(r.get_current_pose())
	 
	 

	 
	#arm_l.set_goal_tolerance(0.01);
	#arm_l.set_num_planning_attempts(1000);

	#arm_l.set_planning_time(10.0);
	plan1 = arm_l.plan()


	arm_l.execute(plan1, wait=True)
	#arm_l.go()
	arm_l.set_pose_target(target_pose)
	#r.set_pose_target(target_pose)
	print(arm_l.get_current_pose())
	#print(r.get_current_pose())

	#print(plan1)
	 
	rospy.sleep(5)

