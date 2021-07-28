#include <ros/ros.h>
#include <rate.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
    
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);  
  //tfBuffer = tf2_ros.Buffer()
  //listener = tf2_ros.TransformListener(tfBuffer)  
  //rate = rospy.Rate(10.0)
  ros::Rate rate(10.0);
 //while (node.ok()){
    try{
       trans = tfBuffer.lookupTransform("camera_rgb_frame", "object_1", ros::Time(0));
      
    catch (tf2_ros::LookupException, tf2_ros::ConnectivityException, tf2_ros::ExtrapolationException){
       ROS_WARN("%s",ex.what());
       ros::Duration(1.0).sleep();
       continue;
       }
    BAr_X = trans.transform.translation.x;
    ROS_INFO(BAr_X);
    BAr_Y = trans.transform.translation.y;
    BAr_Z = trans.transform.translation.z;
    BAr_W = trans.transform.rotation.w;
//tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
   
//wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
      }
   
    move_base_msgs::MoveBaseGoal goal;
   
//we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
   
  goal.target_pose.pose.position.x = BAr_X;//-0.1 #bar_x
  goal.target_pose.pose.position.y = BAr_Y; //#bar_y
  goal.target_pose.pose.position.z = BAr_Z;
  goal.target_pose.pose.orientation.w = BAr_W; //#bar_w
   
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
   
  ac.waitForResult();
   
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the base moved 1 meter forward");
	}  
  else{
    ROS_INFO("The base failed to move forward 1 meter for some reason");
   }
  //}
  return 0;
};
