#!/usr/bin/env python  
import rospy
import os
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import tf
import tf2_ros
import geometry_msgs.msg


def move_base(bar_x, bar_y, bar_z,bar_rx, bar_ry, bar_rz, bar_rw):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = bar_x-0.5
    goal.target_pose.pose.position.y = bar_y+0.1
    goal.target_pose.pose.position.z = bar_z
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

if __name__ == '__main__':
    try:
        rospy.init_node('ar_tag_navigation')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
	x=1
	i=0
	a = np.array(['object_29', 'object_30','object_32'])
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
		os.system("rosnode kill "+ '/explore')
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
	result2= move_base(0.3, 0,0, 0, 0, 0, 1)
	display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
	end_effector_link1 = l.get_end_effector_link()
	end_effector_link = r.get_end_effector_link()
	print(end_effector_link)
	reference_frame = 'base_link'
	try:
		print("check before trans")                
		#trans = tfBuffer.lookup_transform('camera_rgb_optical_frame', 'object_30', rospy.Time())
		trans2 = tfBuffer.lookup_transform('camera_link', a[i], rospy.Time())
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
	x= BAr_Z-0.025 #+ 0.2
	y1= -BAr_X-0.35
	y=  -BAr_X+0.35
	z= 1.12-BAr_Y
	
	if result2:
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
		rospy.sleep(5)
		#catch
		arm_l.set_position_target( [x, -0.32, z] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [x, 0.32, z] ,end_effector_link)
		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go(wait=True)
		rospy.sleep(5)
		#pick
		arm_l.set_position_target( [x, -0.32, (z+0.2)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [x, 0.32, (z+0.2)] ,end_effector_link)
		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go(wait=True)
		rospy.sleep(5)	
		#place
		arm_l.set_position_target( [x, -0.32, z] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [x, 0.32, z] ,end_effector_link)
		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go(wait=True)
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
		print ("ana masheet 30 cm ")
		break

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
