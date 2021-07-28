#!/usr/bin/env python
import rospy
import sys
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float32
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy 
from rospy.numpy_msg import numpy_msg
##from rospy_tutorials.msg import Floats


from std_msgs.msg import Int32 # Messages used in the node must be imported.
'''
"my_callback" is the callback method of the subscriber. Argument "msg" contains the published data.
'''

def my_callback(msg):
	rospy.loginfo("received data from x: %f, y: %f", msg.position.x,msg.position.y)
	group.set_position_target([msg.position.x,msg.position.y,msg.position.z])
	plan = group.go(wait=True)
	group.stop()
	group.clear_pose_targets()


rospy.init_node('pubb') 
omo = rospy.Publisher('/OMOPLATE', Float32, queue_size=10)
rott = rospy.Publisher('/ROTATE', Float32, queue_size=10)
sh = rospy.Publisher('/SHOULDER', Float32, queue_size=10)
bi = rospy.Publisher('/BICEPS', Float32, queue_size=10)
rate = rospy.Rate(10) # 10hz

# moveit start

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm_l"
group = moveit_commander.MoveGroupCommander(group_name)


while not rospy.is_shutdown():
    listener=tf.TransformListener()
    listener.waitForTransform('/base_link','/omo_L',rospy.Time(), rospy.Duration(1.0))
    print listener.frameExists('base_link')
    print listener.frameExists('omo_L')
    (trans,rot)=listener.lookupTransform('base_link','omo_L',rospy.Time())
    x_omo=rot[0]
    y_omo=rot[1]
    z_omo=rot[2]
    w_omo=rot[3]
    print x_omo , y_omo , z_omo ,w_omo 

    #global roll, pitch, yaw
   
    orientation_list0 = [x_omo, y_omo, z_omo, w_omo]
   
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list0)
    print "OMO"
    #print roll*180/3.14
    #print pitch*180/3.14
    #print yaw*180/3.14
    roll0 = abs(roll*180/3.14)
    pitch0 = pitch*180/3.14
    yaw0 = yaw*180/3.14 
   ## theta_O = numpy.array([roll0, pitch0, yaw0], dtype=numpy.float32)
    omo.publish(roll0)
    print roll0
    rate.sleep()

    listener=tf.TransformListener()
    listener.waitForTransform('/shouder_L','/rotate_L',rospy.Time(), rospy.Duration(1.0))
    print listener.frameExists('shouder_L')
    print listener.frameExists('rotate_L')
    (trans,rot)=listener.lookupTransform('shouder_L','rotate_L',rospy.Time())
    x_rotate=rot[0]
    y_rotate=rot[1]
    z_rotate=rot[2]
    w_rotate=rot[3]
    



   
    orientation_list1 = [x_rotate, y_rotate, z_rotate, w_rotate]
    
    (roll, pitch, yaw1) = euler_from_quaternion (orientation_list1)
    print "ROTATE" 
    #print roll*180/3.14
    #print pitch*180/3.14
    #print yaw*180/3.14
    roll = roll*180/3.14
    pitch = pitch*180/3.14
    yaw1 = abs(yaw1*180/3.14)
    ##theta_R = numpy.array([roll, pitch, yaw], dtype=numpy.float32)
    rott.publish(yaw1)
    print yaw1
    rate.sleep()

    listener=tf.TransformListener()
    listener.waitForTransform('/omo_L','/shouder_L',rospy.Time(), rospy.Duration(1.0))
    print listener.frameExists('omo_L')
    print listener.frameExists('shouder_L')
    (trans,rot)=listener.lookupTransform('omo_L','shouder_L',rospy.Time())
    x_shoulder=rot[0]
    y_shoulder=rot[1]
    z_shoulder=rot[2]
    w_shoulder=rot[3]
   
    orientation_list2 = [x_shoulder, y_shoulder, z_shoulder, w_shoulder]
   
    (roll, pitch2, yaw) = euler_from_quaternion (orientation_list2)
    print "SHOULDER" 
    #print roll*180/3.14
    #print pitch*180/3.14
    # print yaw*180/3.14
    roll = roll*180/3.14
    pitch2 = abs(pitch2*180/3.14)
    yaw = yaw*180/3.14
    ##theta_sh = numpy.array([roll, pitch, yaw], dtype=numpy.float32)
    sh.publish(pitch2)
    print pitch2
    rate.sleep()


    listener=tf.TransformListener()
    listener.waitForTransform('/rotate_L','/hand_L',rospy.Time(), rospy.Duration(1.0))
    print listener.frameExists('rotate_L')
    print listener.frameExists('hand_L')
    (trans,rot)=listener.lookupTransform('rotate_L','hand_L',rospy.Time())
    x_biceps=rot[0]
    y_biceps=rot[1]
    z_biceps=rot[2]
    w_biceps=rot[3]
    


   
    orientation_list3 = [x_biceps, y_biceps, z_biceps, w_biceps]
   
    (roll, pitch3, yaw) = euler_from_quaternion (orientation_list3)
    print "BICEPS" 
    #print roll*180/3.14
    #print pitch3*180/3.14
    #print yaw*180/3.14
    roll = roll*180/3.14
    pitch3 = abs(pitch3 *180/3.14)
    yaw = yaw*180/3.14
    ##theta_bi = numpy.array([roll, pitch, yaw], dtype=numpy.float32)
    bi.publish(pitch3)
    print pitch3
    rate.sleep()




    rospy.Subscriber("follow_blob", Pose, my_callback, queue_size=10) 

    rospy.loginfo("subscriber_py node started and subscribed to topic_py")    #debug statement

rospy.spin()
