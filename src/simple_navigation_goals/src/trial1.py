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

pub= rospy.Publisher('Goal1', Int16 , queue_size=10)
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

def callback2(data):
	global sub1
	sub1=data.data
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", sub1)
	if sub1 == 20:
	    print("DONE2")
	    rospy.signal_shutdown("5ara")
if __name__ == '__main__':
    try:
        rospy.init_node('ar_tag_navigation')
        rate = rospy.Rate(10.0)
				x=1
				x1=1
				i=0
				while(x==1):
					result = move_base(0.8,0,0,1)
					if result:
						res= 10              
						rospy.loginfo(res)
						pub.publish(res)
						print("result")
						print (result)
						break
				#while(x1==1):
				print("wait for inverse")
				rospy.Subscriber("Goal2", Int16, callback2)
				rospy.spin()
				print("enter1")
				result1 = move_base(1,0,0,1)
				print("enter2")
				if result1:            
					print("Goal 2 reached")
					print (result)
					break	


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
