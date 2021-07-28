#include <ros.h>
#include <std_msgs/String.h>
// #include <Arduino.h>
#include <Servo.h> 
#include <std_msgs/Int16.h>


ros::NodeHandle  nh;

Servo servo1;
Servo servo2;

std_msgs::Int16 test_1;
std_msgs::Int16 test_2;
ros::Publisher p_servo1("servo_pos1", &test_1);
ros::Publisher p_servo2("servo_pos2", &test_2);

void servo1_cb( const std_msgs::Int16& cmd_msg1){
  servo1.write(cmd_msg1.data); //set servo angle, should be from 0-180  
  delay(10);
  test_1.data = cmd_msg1.data ;
  p_servo1.publish(&test_1); 
}
void servo2_cb( const std_msgs::Int16& cmd_msg2){
  servo2.write(cmd_msg2.data); //set servo angle, should be from 0-180  
  delay(10);
  test_2.data = cmd_msg2.data;
  p_servo2.publish(&test_2); 
}
ros::Subscriber<std_msgs::Int16> sub_1("servo1", &servo1_cb);
ros::Subscriber<std_msgs::Int16> sub_2("servo2", &servo2_cb);
void setup(){
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub_1);
  nh.subscribe(sub_2);
  nh.advertise(p_servo1);
  nh.advertise(p_servo2);
  
  servo1.attach(9); //attach it to pin 9
  servo2.attach(8); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  //delay(1);
}
