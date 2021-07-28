#!/usr/bin/env python  
import rospy
import os
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
import actionlib
import sys
import moveit_commander
import rospy
from std_msgs.msg import String, Int16
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

global end_effector_link1
global end_effector_link
def callback1(data):
	#global x = 0
	sub=data.data
	print(sub)
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", sub)
	if sub == 1:
		print("HOME_P")
		arm_l.go()
		print("PREPARE_P")
		arm_l.set_position_target( [(x+0.04), y1,(z+0.05)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x+0.04), y,(z+0.05)] ,end_effector_link)
		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go(wait=True)
		rospy.sleep(2)        
		print("CATCH_P")
		arm_l.set_position_target( [(x+0.04), (y1+0.2), (z+0.05)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x+0.04), (y-0.2), (z+0.05)] ,end_effector_link)
		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go(wait=True)  
		rospy.sleep(2)            
		print("LIFT_P")
		arm_l.set_position_target( [(x+0.06), (y1+0.2), (z+0.12)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x+0.06),(y-0.2), (z+0.12)] ,end_effector_link)
		#arm_l.set_rpy_target([1.366, 0.297, -1.145],end_effector_link)
		#arm_l.set_rpy_target([-1.366, 0.297, 1.145] ,end_effector_link1)
		arm_l.set_goal_position_tolerance(0.0001)
		#arm_l.set_goal_orientation_tolerance(0.0001)
		arm_l.go(wait=True)		
		rospy.sleep(2)
		pub.publish(2)
		sub=0
		#rospy.signal_shutdown("..")


def callback2(data):
	#global sub3
	sub3=data.data
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", sub3)
	if sub3 == 3:
		rospy.loginfo("Goal 2 Reached!!!")
		print("PLACE_P")
		arm_l.set_position_target( [(x+0.04), (y1+0.2), (z+0.05)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x+0.04), (y-0.2), (z+0.05)] ,end_effector_link)
		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go(wait=True) 
		rospy.sleep(2)         
		print("RELEASE_P")
		arm_l.set_position_target( [(x), -0.4,(1.12)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x), 0.4,(1.12)] ,end_effector_link)
		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go(wait=True)
		rospy.sleep(2)
		rospy.signal_shutdown("..")

rospy.init_node('motion_planning1', anonymous=True)
pub= rospy.Publisher('picking', Int16 , queue_size=10)
moveit_commander.roscpp_initialize(sys.argv)

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
	rospy.Subscriber("Goal1_movebase", Int16, callback1)	
	rospy.Subscriber("Goal2_movebase", Int16, callback2)	
	rospy.spin()
	break

	#break 
		#except rospy.ROSInterruptException:
			#rospy.loginfo("Navigation test finished.")
#rospy.spin()
