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

#pub1= rospy.Publisher('Goal2', Int16 , queue_size=10)
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

def callback1(data):
	global sub
	sub=data.data
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", sub)
	if sub == 10:
	    print("DONE")
	    rospy.signal_shutdown("5ara")

def move_base(x,y,z,w):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x#bar_x-0.5
    goal.target_pose.pose.position.y = y#bar_y+0.1
    goal.target_pose.pose.position.z = z#bar_z
    #goal.target_pose.pose.orientation.x = 0 bar_rz
    #goal.target_pose.pose.orientation.y = 0#bar_ry
    #goal.target_pose.pose.orientation.z = 0#bar_rw #bar_rz
    goal.target_pose.pose.orientation.w = w#-bar_rz #bar_rw
    client.send_goal(goal)
    wait = client.wait_for_result()
    #if not wait:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")
    #else:
    return client.get_result()


while(True):
	try:
	    rospy.Subscriber("Goal1", Int16, callback1)
	    rospy.spin() 
	    print ("pub1")
	    #rospy.sleep(10)
	    #pub1.publish(20)
            #print ("pub1.1")
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
        print("HOME_P")
		arm_l.go()
		arm_l.set_position_target( [(x+0.04), y1,(z+0.05)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x+0.04), y,(z+0.05)] ,end_effector_link)

		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go(wait=True)
        print("PREPARE_P")
	#arm_l.set_rpy_target(  [-1.252, 0.319, 1.006] ,end_effector_link1)
	#arm_l.set_rpy_target(  [-1.183, 0.405, 0.980] ,end_effector_link)
		arm_l.set_goal_orientation_tolerance(0.0001)

	#arm_l.go(wait=True)

	
	#emseekkkk
		arm_l.set_position_target( [(x+0.04), (y1+0.2), (z+0.05)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x+0.04), (y-0.2), (z+0.05)] ,end_effector_link)
		arm_l.set_goal_position_tolerance(0.0001)
		arm_l.go()  
		print("CATCH_P")


	#rospy.Subscriber("FORCE", Int16, callback)

	#rospy.spin() 


	#brfaaaaaaaaaaaaa3
		arm_l.set_position_target( [(x+0.06), (y1+0.2), (z+0.12)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		arm_l.set_position_target( [(x+0.06),(y-0.2), (z+0.12)] ,end_effector_link)

		arm_l.set_goal_position_tolerance(0.0001)

		arm_l.go()
        print("LIFT_P")

		rospy.sleep(5)
	


        #x=1
		#while(x==1):
       	result = move_base(0.75,0,0,1)
        print("MOVEBASE_P")
        if result:              
		    rospy.loginfo("Goal 2 Reached!!!")
		    #place
		    arm_l.set_position_target( [(x+0.04), (y1+0.2), (z+0.05)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		    arm_l.set_position_target( [(x+0.04), (y-0.2), (z+0.05)] ,end_effector_link)
		    arm_l.set_goal_position_tolerance(0.0001)
		    arm_l.go() 
            print("PLACE_P") 
	        #release
		    arm_l.set_position_target( [(x), -0.4,(1.12)] , end_effector_link1) #max[0.55 ,-0.4, 1.2]
		    arm_l.set_position_target( [(x), 0.4,(1.12)] ,end_effector_link)
		    arm_l.set_goal_position_tolerance(0.0001)
		    arm_l.go()
            print("RELEASE_P") 
		    rospy.sleep(5)
		    break

	except rospy.ROSInterruptException:
	    rospy.loginfo("Navigation test finished.")
