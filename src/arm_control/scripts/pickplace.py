#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers
## END_SUB_TUTORIAL

import baxter_interface

from std_msgs.msg import String



class Pick_and_Place(object):


  def __init__(self, limb, side):

  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.


    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    self.group = moveit_commander.MoveGroupCommander("panda_arm")
    self.group.set_max_velocity_scaling_factor(0.7);

    self.gripper = baxter_interface.Gripper("hand")
    self.gripper.calibrate()
    self.gripper.set_holding_force(100.0)

    self.object_pick_pose_target = dict()
    self.object_place_pose_target = dict()
    self.marker = dict()

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.

    self.pub_x = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,queue_size=10)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    print "============ Starting tutorial "

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % self.group.get_planning_frame()
    self.group.set_pose_reference_frame("pedestal")

    ## We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % self.group.get_end_effector_link()

    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print self.robot.get_group_names()




  
  #def pick_place_implementation(self,pose_target):
  def pick_place_implementation(self):
    print "============ picking 1"
    # pose_target = geometry_msgs.msg.Pose()
    # #pose_target.orientation.w = 1.0
    # pose_target.position.x = 0.614834970443
    # pose_target.position.y = -0.273696490998
    # pose_target.position.z = -0.0880582686198

    # pose_target.orientation.x = 0.337657127707
    # pose_target.orientation.y = 0.94104972939
    # pose_target.orientation.z = -0.018643318663
    # pose_target.orientation.w = -0.00809305827207

    # print "adding box"
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = self.robot.get_planning_frame()
    # box_pose.pose.position.x = pose_target.position.x
    # box_pose.pose.position.y = pose_target.position.y
    # box_pose.pose.position.z = pose_target.position.z
    # # box_pose.pose.position.x = 0.137449324269
    # # box_pose.pose.position.y = 0.183503351278
    # # box_pose.pose.position.z = 1.16385190575
    # box_name = "box"
    # self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    index = 0

    while True:
      if index == 4:
        break
      else:

        pick_pose_offset = copy.deepcopy(self.object_pick_pose_target[index])
      
        pick_pose_offset.position.z = pick_pose_offset.position.z + 0.2
        print "pick_pose_offset"
        print pick_pose_offset
        rospy.sleep(3)

        self.moveToPose(pick_pose_offset)

        print "============ grasping open"

        self.gripper.command_position(100.0)

        rospy.sleep(3)

        print "============ picking 2"
        print "pose_target"
        print self.object_pick_pose_target[index]

        self.moveToPose(self.object_pick_pose_target[index])

        print "============ grasping close"

        self.gripper.command_position(0.0)

        rospy.sleep(3)

        print "pick_pose_offset"
        print pick_pose_offset
        print "++++++++++++ going back"

        self.marker_delete(index)

        self.moveToPose(pick_pose_offset)

        print "============ placing 1"
        # place_pose_target = geometry_msgs.msg.Pose()
        # #pose_target.orientation.w = 1.0
        # place_pose_target.position.x = 0.654689623209
        # place_pose_target.position.y = -0.588702890558
        # place_pose_target.position.z = -0.166256389495

        # place_pose_target.orientation.x = 0.451894425385
        # place_pose_target.orientation.y = 0.891042661821
        # place_pose_target.orientation.z = -0.033779026544
        # place_pose_target.orientation.w = -0.0263321189116



        place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])
      
        place_pose_offset.position.z = place_pose_offset.position.z + 0.2
        print "place_pose_offset"
        print place_pose_offset
        rospy.sleep(3)



        self.moveToPose(place_pose_offset)

        #print "============ grasping open"

        #self.gripper.command_position(100.0)

        #rospy.sleep(2)

        print "============ placing 2"
        print "place_pose_target"
        print self.object_place_pose_target[index]

        self.moveToPose(self.object_place_pose_target[index])

        # print "============ grasping close"

        # self.gripper.command_position(0.0)

        # rospy.sleep(2)

        self.marker_add(index,self.object_place_pose_target[index])


        print "place_pose_offset"
        print place_pose_offset
        print "++++++++++++ going back"



        self.moveToPose(place_pose_offset)


        index = index + 1

        ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

        ## END_TUTORIAL

    print "============ STOPPING"

  def moveToPose(self,pose):

        # define temp pose
    pose_target = geometry_msgs.msg.Pose()

        # format the pose correctly
    #print "HELLOOOOOOOOOOO"
    #print pose
    pose_target.orientation.x = pose.orientation.x
    pose_target.orientation.y = pose.orientation.y
    pose_target.orientation.z = pose.orientation.z
    pose_target.orientation.w = pose.orientation.w
    pose_target.position.x = pose.position.x
    pose_target.position.y = pose.position.y
    pose_target.position.z = pose.position.z

        # set things
    self.group.set_pose_target(pose_target)
    self.group.set_num_planning_attempts(5);
    self.group.set_planning_time(10.0);
    self.group.set_goal_position_tolerance(0.0075)
    self.group.set_goal_orientation_tolerance(0.0075)

    print("\tPlanning...")
    plan1 = self.group.plan()
        # rospy.sleep(5)
    print("\tExecuting...")
    self.group.go(wait=True)

  def input_pos(self,filename):
    index = 0



    with open(filename, 'r') as f:
      while True:
        if index == 4:
          break
        else:
        
          line = f.readline()
          line = line.rstrip()
          data = line.split(',')
          print data
          self.object_pick_pose_target[index] = geometry_msgs.msg.Pose()
          self.object_pick_pose_target[index].position.x = float(data[0])
          self.object_pick_pose_target[index].position.y = float(data[1])
          self.object_pick_pose_target[index].position.z = float(data[2])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')
          print data
          self.object_pick_pose_target[index].orientation.x = float(data[0])
          self.object_pick_pose_target[index].orientation.y = float(data[1])
          self.object_pick_pose_target[index].orientation.z = float(data[2])
          self.object_pick_pose_target[index].orientation.w = float(data[3])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')

          self.object_place_pose_target[index] = geometry_msgs.msg.Pose()
          self.object_place_pose_target[index].position.x = float(data[0])
          self.object_place_pose_target[index].position.y = float(data[1])
          self.object_place_pose_target[index].position.z = float(data[2])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')
          
          self.object_place_pose_target[index].orientation.x = float(data[0])
          self.object_place_pose_target[index].orientation.y = float(data[1])
          self.object_place_pose_target[index].orientation.z = float(data[2])
          self.object_place_pose_target[index].orientation.w = float(data[3])

          print self.object_pick_pose_target[index]
          print self.object_place_pose_target[index]

          index = index + 1

        # print "adding box"
    rospy.sleep(2)
    
    #rate.sleep()
        
    #self.vel = vel



    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = self.robot.get_planning_frame()
    # box_pose.pose.position.x = self.object_pick_pose_target[0].position.x 
    # box_pose.pose.position.y = self.object_pick_pose_target[0].position.y
    # box_pose.pose.position.z = self.object_pick_pose_target[0].position.z
    # # box_pose.pose.position.x = 0.137449324269
    # # box_pose.pose.position.y = 0.183503351278
    # # box_pose.pose.position.z = 1.16385190575
    # box_name = "box"
    # self.scene.add_box(box_name, box_pose, size=(0.08, 0.08, 0.08))

  def marker_add(self,index,pose_target):
 
    self.marker[index] = Marker()
    self.marker[index].header.frame_id = self.robot.get_planning_frame()
    self.marker[index].header.stamp = rospy.Time.now()
    self.marker[index].ns = "robot"
    self.marker[index].action = self.marker[index].ADD
    self.marker[index].type = self.marker[index].SPHERE
    self.marker[index].id = index

    self.marker[index].scale.x = 0.1
    self.marker[index].scale.y = 0.1
    self.marker[index].scale.z = 0.1
        
            #
        
    self.marker[index].color.a = 1.0
    self.marker[index].color.r = 0.0
    self.marker[index].color.g = 1.0 
    self.marker[index].color.b = 0.0

        # vel.pose.position.x = 1;
        # vel.pose.position.y = 1;
        # vel.pose.position.z = 1;
        # vel.pose.orientation.x = 0.0;
        # vel.pose.orientation.y = 0.0;
        # vel.pose.orientation.z = 0.0;
        # vel.pose.orientation.w = 1.0;

    self.marker[index].pose.position.x = pose_target.position.x
    self.marker[index].pose.position.y = pose_target.position.y
    self.marker[index].pose.position.z = pose_target.position.z
    self.marker[index].pose.orientation.x = pose_target.orientation.x 
    self.marker[index].pose.orientation.y = pose_target.orientation.y 
    self.marker[index].pose.orientation.z = pose_target.orientation.z 
    self.marker[index].pose.orientation.w = pose_target.orientation.w 

        



    

    
        #vel.text = "Hello"
    print "check\n\n\n\n\n\n\n"
      
    self.pub_x.publish(self.marker[index])
      #rate = rospy.Rate(1)
    print "sending marker", self.marker[index]
        

  def marker_delete(self,index):
    self.marker[index].action = self.marker[index].DELETE
    self.pub_x.publish(self.marker[index])


  def ar_tag_callback(self,data):

    pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.w = 1.0

    print data.markers[0].id 

    for x in range(12):
      if data.markers[x].id == 1:
        print data.markers[x].pose.pose.position.z
        pose_target.position.x = data.markers[x].pose.pose.position.x
        pose_target.position.y = data.markers[x].pose.pose.position.y
        pose_target.position.z =  data.markers[x].pose.pose.position.z

        pose_target.orientation.x = data.markers[x].pose.pose.orientation.x
        pose_target.orientation.y = data.markers[x].pose.pose.orientation.y
        pose_target.orientation.z = data.markers[x].pose.pose.orientation.z
        pose_target.orientation.w = data.markers[x].pose.pose.orientation.w

    rospy.sleep(2)
    print "adding box"
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_pose.pose.position.x = pose_target.position.x
    box_pose.pose.position.y = pose_target.position.y
    box_pose.pose.position.z = pose_target.position.z
    # box_pose.pose.position.x = 0.137449324269
    # box_pose.pose.position.y = 0.183503351278
    # box_pose.pose.position.z = 1.16385190575
    box_name = "box"
    self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_pose.pose.position.x = 0.505527106173
    box_pose.pose.position.y = -0.512224242973
    box_pose.pose.position.z = -0.162671165001
    box_name = "boxx"
    self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    self.pick_place_implementation(pose_target)
    # print data.markers[1].id
    # print data.markers[0].pose.pose.position.x
    # print data.markers[1].pose.pose.position.x

def main():
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  ## BAnima - enabling robot
  
  rs = baxter_interface.RobotEnable()
  rs.enable()
  pickplace = Pick_and_Place('left_arm','left')

  #sub_img = rospy.Subscriber("ar_pose_marker", AlvarMarkers, pickplace.ar_tag_callback)
  pickplace.input_pos("position_record.txt")
  for index in range(0, 4):
    pickplace.marker_add(index,pickplace.object_pick_pose_target[index]) #sending 1 in case of pick position
  pickplace.pick_place_implementation()

  rospy.spin()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
