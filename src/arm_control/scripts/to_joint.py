#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("motion_planning")

scene = PlanningSceneInterface()
robot = RobotCommander()
arm_l = MoveGroupCommander("arm_l")


display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    DisplayTrajectory)

group_variable_values = arm_l.get_current_joint_values()
print "============ Joint values: ", group_variable_values
group_variable_values[0] = 0.2
arm_l.set_joint_value_target(group_variable_values)
 
 
plan1=arm_l.plan()
arm_l.execute(plan1)
rospy.sleep(5)
