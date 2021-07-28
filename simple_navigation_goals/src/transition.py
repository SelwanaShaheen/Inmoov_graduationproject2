#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def move_base(bar_x, bar_y, bar_w):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.0#bar_x
    goal.target_pose.pose.position.y = 1.0#bar_y
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.w = bar_w
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('ar_tag_navigation')

        tfBuffer = tf.TransformBroadcaster()
        listener = tf.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
	listener.waitForTransform("/camera_rgb_frame", "/object_1", rospy.Time(), rospy.Duration(4.0))
        while not rospy.is_shutdown():
            try:
		now = rospy.Time.now()
		listener.waitForTransform("/camera_rgb_frame", "/object_1", now, rospy.Duration(4.0))
                trans = listener.lookupTransform('/camera_rgb_frame', '/object_1', now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rate.sleep()
                continue

            BAr_X = trans.transform.translation.x
            BAr_Y = trans.transform.translation.z
	    BAr_W = trans.transform.rotation.w

            result = move_base(BAr_X, BAr_Y, BAr_W)
            if result:
                rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
