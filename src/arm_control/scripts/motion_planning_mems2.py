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




moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion_planning1', anonymous=True)
scene = PlanningSceneInterface()
robot = RobotCommander()
arm_l = MoveGroupCommander("both_arms")
l = MoveGroupCommander("arm_l")
r = MoveGroupCommander("arm_r")
 
 
display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)

 
end_effector_link1 = l.get_end_effector_link()
end_effector_link = r.get_end_effector_link()
print(end_effector_link)
print(end_effector_link1)
reference_frame = 'base_link'
 
 
arm_l.allow_replanning(True)

target_pose = PoseStamped()
target_pose1 = PoseStamped()
home_l = PoseStamped()
home_r = PoseStamped()
arm_l.set_pose_reference_frame(reference_frame)
#target_pose.header.frame_id = reference_frame
#target_pose.header.stamp = rospy.Time.now()
target_pose.pose.position.x = 0.154000
target_pose.pose.position.y = 0.413000
target_pose.pose.position.z = 1.251000
target_pose.pose.orientation.x = 0.557000 
target_pose.pose.orientation.y = 0.137000  
target_pose.pose.orientation.z = -0.516000  
target_pose.pose.orientation.w = 0.636000  
#arm_l.set_position_target(target_pose)

target_pose1.pose.position.x = 0.154000
target_pose1.pose.position.y = -0.413000
target_pose1.pose.position.z = 1.251000
target_pose1.pose.orientation.x = -0.557000  
target_pose1.pose.orientation.y = -0.137000  
target_pose1.pose.orientation.z = 0.516000  
target_pose1.pose.orientation.w = 0.636000  
#arm_l.set_named_target('goal2')
arm_l.go()
rospy.sleep(2)
 
arm_l.set_num_planning_attempts(100000);

#arm_l.go()
arm_l.set_position_target([0.475, -0.293, 1.061], end_effector_link1)
arm_l.set_position_target([0.475, 0.293, 1.061] ,end_effector_link)
#arm_l.set_goal_tolerance(0.002)
arm_l.set_goal_position_tolerance(0.0001)
arm_l.go(wait=True)

#arm_l.set_rpy_target([1.108, 0.026, -1.133],end_effector_link)
#arm_l.set_rpy_target([-1.108, 0.026, 1.133] ,end_effector_link1)
#arm_l.set_goal_orientation_tolerance(0.09)

#arm_l.go(wait=True)
print l.get_current_pose()
print r.get_current_pose()

#arm_l.set_pose_target(target_pose1, end_effector_link1)
#arm_l.set_pose_target(target_pose, end_effector_link)

#arm_l.set_pose_target(target_pose, end_effector_link)
#arm_l.set_goal_tolerance(0.5)
#r.set_pose_target(target_pose1)
#arm_l.set_pose_target(target_pose, "hand_R")
#arm_l.set_pose_target(target_pose, "hand_L")
#arm_l.go()
#l.go()
#r.go()

def callback(data):
    global force 
    force = data.data
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
    if force > 500:
        print ("ana f if")
	group_variable_values = arm_l.get_current_joint_values()
	print(group_variable_values)
	group_variable_values[0] = group_variable_values[0] - 0.29
	group_variable_values[1] = group_variable_values[1] + 0.494
	group_variable_values[3] = 0 
	group_variable_values[5] = group_variable_values[5]  + 0.6
	group_variable_values[6] = group_variable_values[6] - 0.45
	group_variable_values[8] = 0
	#group_variable_values_right [1] = -0.5
	#group_variable_values_right [0] = -0.2
	arm_l.set_joint_value_target(group_variable_values)
	#r.set_joint_value_target(group_variable_values_right)
	arm_l.set_goal_position_tolerance(0.0001)
	arm_l.go()
	print(group_variable_values)
	#group_variable_values_right = r.get_current_joint_values()

	arm_l.set_position_target([0.475, -0.293, 1.061], end_effector_link1)
	arm_l.set_position_target([0.475, 0.293, 1.061] ,end_effector_link)
	#arm_l.set_goal_tolerance(0.002)
	arm_l.set_goal_position_tolerance(0.0001)
	arm_l.go()


	#r.set_joint_value_target(group_variable_values_right)
	#r.set_goal_position_tolerance(0.0001)
	#r.go()

	group_variable_values = arm_l.get_current_joint_values()
	print(group_variable_values)
	 
	group_variable_values[1] =  0.1
	group_variable_values[6] = -0.15

	arm_l.set_joint_value_target(group_variable_values)
	arm_l.set_goal_position_tolerance(0.0001)
	arm_l.go()
        rospy.signal_shutdown("5ara")



    
print ("2abl")
rospy.Subscriber("FORCE", Int16, callback)
print ("ba3d")
rospy.spin() 



 


#while not rospy.is_shutdown():

print ("gowa")

