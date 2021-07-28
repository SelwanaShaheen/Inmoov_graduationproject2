#!/usr/bin/env python

import sys
import moveit_commander
import rospy
from std_msgs.msg import String, Int16
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

rospy.init_node('motion_planning1', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

tfBuffer = tf2_ros.Buffer() 				
listener = tf2_ros.TransformListener(tfBuffer)		
rate = rospy.Rate(10.0)					

scene = PlanningSceneInterface()
robot = RobotCommander()
arm_l = MoveGroupCommander("both_arms")
l = MoveGroupCommander("arm_l")
r = MoveGroupCommander("arm_r")

	

def callback(data):
	    global force	
	    force = data.data
	    rospy.loginfo("FORCE= %s",force)
	    if force > 500:
		#erfaaa3333
		print ("brfaaaaaaaa3")
		arm_l.set_position_target( [(x+0.03), (y1+0.08), (z+0.05)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x+0.03),(y-0.08), (z+0.05)] ,end_effector_link)
		#arm_l.set_rpy_target([1.366, 0.297, -1.145],end_effector_link)
		#arm_l.set_rpy_target([-1.366, 0.297, 1.145] ,end_effector_link1)
		arm_l.set_goal_position_tolerance(0.0001)
		#arm_l.set_goal_orientation_tolerance(0.0001)
		arm_l.go()
		rospy.signal_shutdown("5ara")
     	


	
while(True):
		     

	display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
	end_effector_link1 = l.get_end_effector_link()
	end_effector_link = r.get_end_effector_link()
	reference_frame = 'base_link'
	x1=0

	if x1 > 1:
		try:
			print("check before trans")                
			trans = tfBuffer.lookup_transform('camera_rgb_optical_frame', 'object_49', rospy.Time())
			#trans = tfBuffer.lookup_transform('camera_frame_optical', 'object_39', rospy.Time())
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

	
	arm_l.set_num_planning_attempts(10000);
	
	if x1 > 1:
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
	#x= BAr_Z #+ 0.2
	#y1= -BAr_X-0.35
	#y=  -BAr_X+0.35
	#z= 1.17#-BAr_Y
	x=0.51
	y1=-0.38
	y=0.38
	z=1.12

	



	arm_l.go()
	arm_l.set_position_target( [x, y1, z] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
	arm_l.set_position_target( [x, y, z] ,end_effector_link)

	arm_l.set_goal_position_tolerance(0.0001)
	arm_l.go(wait=True)

	#arm_l.set_rpy_target(  [-1.252, 0.319, 1.006] ,end_effector_link1)
	#arm_l.set_rpy_target(  [-1.183, 0.405, 0.980] ,end_effector_link)
	arm_l.set_goal_orientation_tolerance(0.0001)

	#arm_l.go(wait=True)
	print l.get_current_pose()
	print r.get_current_pose()
	
	#emseekkkk
	arm_l.set_position_target( [x, (y1+0.08), z] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
	arm_l.set_position_target( [x, (y-0.08), z] ,end_effector_link)
	arm_l.set_goal_position_tolerance(0.0001)
	arm_l.go()  
		
  	print ("henaaaaaaaaa")

	#rospy.Subscriber("FORCE", Int16, callback)
	print ("ba3d")
	#rospy.spin() 



	arm_l.set_position_target( [(x+0.02), (y1+0.08), (z+0.09)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
	arm_l.set_position_target( [(x+0.02),(y-0.08), (z+0.09)] ,end_effector_link)
	#arm_l.set_rpy_target([1.366, 0.297, -1.145],end_effector_link)
	#arm_l.set_rpy_target([-1.366, 0.297, 1.145] ,end_effector_link1)
	arm_l.set_goal_position_tolerance(0.0001)
	#arm_l.set_goal_orientation_tolerance(0.0001)
	arm_l.go()


	rospy.sleep(5)
	

	print ("henaaaaaaaaasssssssssssssssssssssssssssssssssssssssssssssss")
	
	#place
	arm_l.set_position_target( [x,(y1+0.08), z] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
	arm_l.set_position_target( [x, (y-0.08), z] ,end_effector_link)
	arm_l.set_goal_position_tolerance(0.0001)
	arm_l.go(wait=True)
	#release
	arm_l.set_position_target( [(x), y1,z] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
	arm_l.set_position_target( [(x), y,z] ,end_effector_link)
	arm_l.set_goal_position_tolerance(0.0001)
	arm_l.go()


	#arm_l.set_position_target( [0.339, -0.270, 0.774] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
	#arm_l.set_position_target( [0.395, 0.270, 0.772] ,end_effector_link)
	#arm_l.set_goal_position_tolerance(0.0001)
	#arm_l.set_rpy_target(  [-0.605, 0.233, 0.707] ,end_effector_link1)
	#arm_l.set_rpy_target(  [0.608, 0.226, -0.709] ,end_effector_link)
	#arm_l.set_goal_position_tolerance(0.0001)
	#arm_l.go()




	#group_variable_values = arm_l.get_current_joint_values()
	#print(group_variable_values)
	#group_variable_values[0] = group_variable_values[0] - 0.29
	#group_variable_values[1] = group_variable_values[1] + 0.494
	#group_variable_values[3] = 0 
	#group_variable_values[5] = group_variable_values[5]  + 0.6
	#group_variable_values[6] = group_variable_values[6] - 0.45
	#group_variable_values[8] = 0
	#group_variable_values_right [1] = -0.5
	#group_variable_values_right [0] = -0.2
	#arm_l.set_joint_value_target(group_variable_values)
	#r.set_joint_value_target(group_variable_values_right)
	#arm_l.set_goal_position_tolerance(0.0001)
	#arm_l.go()
	#print(group_variable_values)
	#group_variable_values_right = r.get_current_joint_values()

	#arm_l.set_position_target([0.475, -0.293, 1.061], end_effector_link1)
	#arm_l.set_position_target([0.475, 0.293, 1.061] ,end_effector_link)
	#arm_l.set_goal_tolerance(0.002)
	#arm_l.set_goal_position_tolerance(0.0001)
	#arm_l.go()


	#r.set_joint_value_target(group_variable_values_right)
	#r.set_goal_position_tolerance(0.0001)
	#r.go()

	#group_variable_values = arm_l.get_current_joint_values()
	#print(group_variable_values)
	 
	#group_variable_values[1] =  0.1
	#group_variable_values[6] = -0.15

	#arm_l.set_joint_value_target(group_variable_values)
	#arm_l.set_goal_position_tolerance(0.0001)
	#arm_l.go()
	 
	rospy.sleep(5)
	break
