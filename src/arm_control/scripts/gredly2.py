#!/usr/bin/env python

import sys
import moveit_commander
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
import numpy as np
import math
import tf
import tf2_ros
import geometry_msgs.msg


#def move_it_L(bar_x, bar_y, bar_z, bar_rx, bar_ry, bar_rz, bar_rw):
#	target_pose = PoseStamped()
	#target_pose.header.frame_id = reference_frame
	#target_pose.header.stamp = rospy.Time.now()
#	target_pose.pose.position.x = (bar_z + 0.2)
#	target_pose.pose.position.y = -(bar_x - 0.2)
#	target_pose.pose.position.z = -bar_y + 1.12
#	target_pose.pose.orientation.x = -bar_rz
#	target_pose.pose.orientation.y =  -bar_rx
#	target_pose.pose.orientation.z = -bar_ry
#	target_pose.pose.orientation.w = -bar_rw
#	return target_pose

#def move_it_R(bar_x, bar_y, bar_z, bar_rx, bar_ry, bar_rz, bar_rw):
#	target_pose1 = PoseStamped()
#	target_pose1.pose.position.x =  (bar_z + 0.2)
#	target_pose1.pose.position.y = -(bar_x + 0.2)
#	target_pose1.pose.position.z = -bar_y + 1.12
#	target_pose1.pose.orientation.x = bar_rz
#	target_pose1.pose.orientation.y = -bar_rx
#	target_pose1.pose.orientation.z = -bar_ry
#	target_pose1.pose.orientation.w =  bar_rw
#	return target_pose1


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion_planning1', anonymous=True)
tfBuffer = tf2_ros.Buffer() 				
listener = tf2_ros.TransformListener(tfBuffer)		
rate = rospy.Rate(10.0)					
x=1
								
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
		trans = tfBuffer.lookup_transform('camera_frame_optical', 'object_47', rospy.Time())
		print("check passed trans")
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
	    	print("check unpassed trans")                
	    	rate.sleep()
            	continue

	#r.set_pose_reference_frame(reference_frame)
	arm_l.allow_replanning(True)
	#r.allow_replanning(True)
	target_pose = PoseStamped()
	target_pose1 = PoseStamped()
	arm_l.set_pose_reference_frame(reference_frame)
	#if (BAr_Z != 0): #and BAr_Z <= 0.3):
										#g
	    	#target_pose=move_it_L(BAr_X, BAr_Y, BAr_Z, BAr_rX, BAr_rY, BAr_rZ, BAr_rW)			#g
	    	#target_pose1=move_it_R(BAr_X, BAr_Y, BAr_Z, BAr_rX, BAr_rY, BAr_rZ, BAr_rW)			#g
		#print(arm_l.get_current_pose())
		#arm_l.set_goal_position_tolerance(0.5)
		#arm_l.set_goal_orientation_tolerance(0.5)
		
	BAr_X = trans.transform.translation.x
	print(BAr_X)
	BAr_Y = trans.transform.translation.y
	print(BAr_Y)
	BAr_Z = trans.transform.translation.z
	print(BAr_Z)
	BAr_rX = trans.transform.rotation.x
	#print(BAr_rX)	
	BAr_rY = trans.transform.rotation.y
	#print(BAr_rY)
	BAr_rZ = trans.transform.rotation.z
	#print(BAr_rZ)
	BAr_rW = trans.transform.rotation.w	
	#print(BAr_rW)	
	orientation_list = [BAr_rX, BAr_rY, BAr_rZ, BAr_rW]
 	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	print(orientation_list)
	print (roll)
	print (pitch)
	print (yaw)
	pose_x= BAr_Y
        pose_y=- BAr_Z
        pose_z= -BAr_X + 0.2

        camera_x= pose_z
        camera_y= - pose_x
        camera_z= - pose_y
	#target_pose.pose.position.x = (BAr_Z) + 0.2 #0.116 
	#target_pose.pose.position.y =  (BAr_X)+0.4 #0.243
	#target_pose.pose.position.z =  -BAr_Y + 1.12 #1.035 
	#target_pose.pose.orientation.x =-BAr_rZ  #0.163 
	#target_pose.pose.orientation.y = -BAr_rX #-0.001 
	#target_pose.pose.orientation.z =  BAr_rY #0.186 
	#target_pose.pose.orientation.w =  -BAr_rW #-0.969 
	#arm_l.set_position_target(target_pose)
	rospy.sleep(2)
	arm_l.set_num_planning_attempts(10000);
	arm_l.go()
	pos1 = arm_l.set_position_target([camera_x,-camera_y, camera_z+1.12], end_effector_link1)
	#pos2 = arm_l.set_position_target([(BAr_Z) + 0.2,(BAr_X)+0.4, -BAr_Y + 1.12] ,end_effector_link)

         
	arm_l.set_goal_position_tolerance(0.2)
	arm_l.go(wait=True)
	
	#arm_l.set_rpy_target( [abs(roll),abs(pitch),-abs(yaw)] ,end_effector_link)
	#arm_l.set_rpy_target( [-abs(roll),abs(pitch),abs(yaw)] ,end_effector_link1)
	#arm_l.set_goal_orientation_tolerance(0.1)
	#arm_l.go(wait=True)
	 
	#target_pose1.pose.position.x = (BAr_Z) + 0.2
	#target_pose1.pose.position.y = (BAr_X)-0.4
	#target_pose1.pose.position.z = -BAr_Y + 1.12
	#target_pose1.pose.orientation.x = BAr_rZ
	#target_pose1.pose.orientation.y = -BAr_rX
	#target_pose1.pose.orientation.z = BAr_rY
	#target_pose1.pose.orientation.w = BAr_rW
			#arm_l.set_named_target('goal2')
	#print(target_pose)
	#print(target_pose1)
	#arm_l.go()
	#r.go()
	#rospy.sleep(2)
	#arm_l.set_num_planning_attempts(10000);
	#r.set_num_planning_attempts(10000);
	#arm_l.set_pose_target(target_pose1, end_effector_link1)
	#arm_l.set_pose_target(target_pose, end_effector_link)
	#arm_l.set_goal_tolerance(0.5)
	#r.set_goal_tolerance(0.5)
	#arm_l.go()
	#r.go()
	rospy.sleep(5)
	break
