#!/usr/bin/env python

import sys
import moveit_commander
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion_planning1', anonymous=True)

scene = PlanningSceneInterface()
robot = RobotCommander()
arm_l = MoveGroupCommander("R_arm")


display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
 
end_effector_link = arm_l.get_end_effector_link()
reference_frame = 'base_link'
arm_l.set_pose_reference_frame(reference_frame)
arm_l.allow_replanning(True)

print(arm_l.get_current_pose())
arm_l.set_goal_position_tolerance(0.5)
arm_l.set_goal_orientation_tolerance(0.000005);

#target_pose = Pose()
#target_pose.header.frame_id = reference_frame
#target_pose.header.stamp = rospy.Time.now()
#target_pose.position.x =  -0.0572300629886
#target_pose.position.y = -0.194748492739
#target_pose.position.z = 0.627917957336
#target_pose.pose.orientation.x = 0.0605532236567
#target_pose.pose.orientation.y = -0.350735920434
#target_pose.pose.orientation.z = 0.152864566593
#target_pose.orientation.w = 0.921927353701
#arm_l.set_position_target(target_pose)


arm_l.set_named_target('goal2')
arm_l.go()
rospy.sleep(2)
               
print(arm_l.get_current_pose())

 
 

 
#arm_l.set_goal_tolerance(0.01);
#arm_l.set_num_planning_attempts(1000);

#arm_l.set_planning_time(10.0);
plan1 = arm_l.plan()


arm_l.execute(plan1, wait=True)
#arm_l.go()
#arm_l.set_pose_target(target_pose)
print(arm_l.get_current_pose())

#print(plan1)
 
rospy.sleep(5)


