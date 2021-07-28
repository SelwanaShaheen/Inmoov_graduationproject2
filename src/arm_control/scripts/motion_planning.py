#!/usr/bin/env python

import sys
import moveit_commander
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion_planning1', anonymous=True)

scene = PlanningSceneInterface()
robot = RobotCommander()
arm_l = MoveGroupCommander("panda_arm")


display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
 
end_effector_link = arm_l.get_end_effector_link()
reference_frame = 'panda_link0'
arm_l.set_pose_reference_frame(reference_frame)
arm_l.allow_replanning(True)


arm_l.set_goal_position_tolerance(0.5)
arm_l.set_goal_orientation_tolerance(0.000005);

arm_l.set_named_target('home')
arm_l.go()
rospy.sleep(2)
               
print(arm_l.get_current_pose())

print "============ Generating plan 1"
target_pose = PoseStamped()
 
 
target_pose.header.frame_id = reference_frame
target_pose.header.stamp = rospy.Time.now()     
target_pose.pose.position.x = 0.0739513
target_pose.pose.position.y = -0.2197271

target_pose.pose.position.z = 1.032437
target_pose.pose.orientation.x = 0.0
target_pose.pose.orientation.y = 0.0
target_pose.pose.orientation.z = 0.0
target_pose.pose.orientation.w = 1.2010140

arm_l.set_start_state_to_current_state()
arm_l.set_position_target([0.2,-0.2,0.7],end_effector_link)


 
#arm_l.set_goal_tolerance(0.01);
#arm_l.set_num_planning_attempts(1000);

#arm_l.set_planning_time(10.0);
plan1 = arm_l.plan()


arm_l.execute(plan1, wait=True)
arm_l.go()

 
rospy.sleep(5)


