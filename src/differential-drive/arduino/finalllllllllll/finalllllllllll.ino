#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif


#include <ros.h>
#include <std_msgs/String.h>
// #include <Arduino.h>
#include <Servo.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#define FSR_pin A8 

#define PWM_L 11
#define DIR_L 12
///////right motor//////
#define PWM_R 9
#define DIR_R 8
//// Right encoder////////
#define RH_ENCODER_A  21
#define RH_ENCODER_B  20
//left encoder///////////
#define LH_ENCODER_A 2
#define LH_ENCODER_B 3


int reading ;
volatile  long int rightCounter = 0; 
volatile  long int leftCounter = 0; 


ros::NodeHandle  nh;

Servo OMOPLATE_L ;
Servo OMOPLATE_R ;
Servo SHOULDER_L ;
Servo SHOULDER_R ;
Servo ROTATE_L ;
Servo ROTATE_R ;
Servo BICEPS_L ;
Servo BICEPS_R ;

std_msgs::Int16 Force_msg;
std_msgs::Float32 test_1;
std_msgs::Float32 test_2;
std_msgs::Float32 test_3;
std_msgs::Float32 test_4;
std_msgs::Int16 len_msg;
std_msgs::Int16 ren_msg;

std_msgs::Int16 inversss;

ros::Publisher p_OMOPLATE("OMOPLATE_pos", &test_1);
ros::Publisher p_SHOULDER("SHOULDER_pos", &test_2);
ros::Publisher p_ROTATE("ROTATE_pos", &test_3);
ros::Publisher p_BICEPS("BICEPS_pos", &test_4);
ros::Publisher pub_Force("FORCE",&Force_msg);
ros::Publisher pub_lencoder( "lwheel", &len_msg);
ros::Publisher pub_rencoder( "rwheel", &ren_msg);

ros::Publisher pub_inverse( "picking", &inversss);






void OMOPLATE_cb( const std_msgs::Float32& cmd_msg1){
  float omoplate_L = map(cmd_msg1.data , 0, 90,65, 10);
  float omoplate_R = map(cmd_msg1.data +10 , 0, 90,163,135);
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  delay(10);
  test_1.data = cmd_msg1.data ;
  p_OMOPLATE.publish(&test_1); 
}
void SHOULDER_cb( const std_msgs::Float32& cmd_msg2){
  float shoulder_L = map( cmd_msg2.data , 0, 90, 175,90);
  float shoulder_R = map( cmd_msg2.data , 0, 90, 170,40);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R);  
  delay(10);
  test_2.data = cmd_msg2.data;
  p_SHOULDER.publish(&test_2); 
}
void ROTATE_cb( const std_msgs::Float32& cmd_msg3){
  float rotate_L = map(cmd_msg3.data-10, 0, 90, 120, 180);
  float rotate_R = map(cmd_msg3.data, 0, 90, 60, 90);
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
  delay(10);
  test_3.data = cmd_msg3.data ;
  p_ROTATE.publish(&test_3); 
}
void BICEPS_cb( const std_msgs::Float32& cmd_msg4){
  float biceps_L = map(cmd_msg4.data, 0, 90, 70, 150);
  float biceps_R = map(cmd_msg4.data, 0, 90, 120, 40);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  delay(10);
  test_4.data = cmd_msg4.data ;
  p_BICEPS.publish(&test_4); 
}

void inverse_( const std_msgs::Int16& cmd_msg5){

  int sub = cmd_msg5.data ;
  if ( sub == 1 ){
   PREPARE();
   delay(10000);
   CATCH();
   delay(10000);
   LIFT();
   inversss.data = 2;
   pub_inverse.publish(&inversss);
   sub = 0 ;
  }
}


void inverse_1_( const std_msgs::Int16& cmd_msg6){

  int sub = cmd_msg6.data ;
  if ( sub == 3 ){
   CATCH();
   delay(10000);
   RELEASE();
   sub = 0 ;
  }
}



void PREPARE (){
  float biceps_L = map(0, 0, 90, 70, 150);
  float biceps_R = map(0, 0, 90, 115, 40);
  float rotate_L = map(0, 0, 90, 100, 130);
  float rotate_R = map(0, 0, 90, 75, 95);
  float shoulder_L = map( 45, 0, 90, 175,90);
  float shoulder_R = map(45, 0, 90, 170,40);
  float omoplate_L = map(45 , 0, 90,60, 5);
  float omoplate_R = map(45, 0, 90,125,145);
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  delay(2000);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R); 
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
}


void CATCH (){
  float biceps_L = map(50, 0, 90, 70, 150);
  float biceps_R = map(50, 0, 90, 115, 40);
  float rotate_L = map(0, 0, 90, 100, 130);
  float rotate_R = map(0, 0, 90, 75, 95);
  float shoulder_L = map( 45, 0, 90, 175,90);
  float shoulder_R = map(45, 0, 90, 170,40);
  float omoplate_L = map(0 , 0, 90,60, 5);
  float omoplate_R = map(0, 0, 90,125,145);
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  delay(3000);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R); 
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
}


void LIFT (){
  float biceps_L = map(60, 0, 90, 70, 150);
  float biceps_R = map(60, 0, 90, 115, 40);
  float rotate_L = map(0, 0, 90, 100, 130);
  float rotate_R = map(0, 0, 90, 75, 95);
  float shoulder_L = map( 65, 0, 90, 175,90);
  float shoulder_R = map(65, 0, 90, 170,40);
  float omoplate_L = map(0, 0, 90,60, 5);
  float omoplate_R = map(0,  0, 90,125,145);
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
  delay(2000);
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R); 
  
}


void RELEASE (){
  float biceps_L = map(0, 0, 90, 70, 150);
  float biceps_R = map(0, 0, 90, 115, 40);
  float rotate_L = map(0, 0, 90, 100, 130);
  float rotate_R = map(0, 0, 90, 75,95);
  float shoulder_L = map( 0 , 0, 90, 175,90);
  float shoulder_R = map(0, 0, 90, 170,40);
  float omoplate_L = map(0 , 0, 90,60, 5);
  float omoplate_R = map(0, 0, 90,125,145);
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R); 
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
}

 
 

void servo_cb(const sensor_msgs::JointState& cmd_msg){

  float OMOPLATE = radiansToDegrees(cmd_msg.position[4]);
  float omoplate_L = map(OMOPLATE, 0, 90,65, 5);
  float omoplate_R = map(OMOPLATE, 0, 90,125,145);
  Serial.print(OMOPLATE); 

  float SHOULDER = radiansToDegrees(cmd_msg.position[5]);
  float shoulder_L = map(SHOULDER, 0, 90, 175,90);
  float shoulder_R = map(SHOULDER, 0, 90, 170,40);

  float ROTATE = radiansToDegrees(cmd_msg.position[6]);
  float rotate_L = map(ROTATE, 0, 90, 100, 130);
  float rotate_R = map(ROTATE, 0, 90, 75,95);

  float BICEBS = radiansToDegrees(cmd_msg.position[7]);
  float biceps_L = map(BICEBS, 0, 90, 70, 150);
  float biceps_R = map(BICEBS, 0, 90, 115, 40);
  
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R); 
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
}



void lmotor_Cb( const std_msgs::Float32& lm){
  
  left_motor_control(lm.data);
}

void rmotor_Cb( const std_msgs::Float32& rm){
  
  right_motor_control(rm.data);
}



void left_motor_control(float vel1){
 //move the wheel Forward
  if(vel1> 0.0){
   digitalWrite(DIR_L,HIGH);
   analogWrite(PWM_L,vel1);
   
    }
    
   //move the wheel backward
  else if(vel1< 0.0){
     vel1=abs(vel1);
     digitalWrite(DIR_L,LOW);
     analogWrite(PWM_L,vel1);
   // Serial.println("Backward");
    }
   else if(vel1== 0) {    
     digitalWrite(PWM_L, vel1);
      }
  }
  
  void right_motor_control(float vel2){
 //move the wheel Forward
  if(vel2> 0.0){
    digitalWrite(DIR_R,HIGH);
   analogWrite(PWM_R,vel2);
    }
   //move the wheel backward
  else if(vel2< 0.0){
    vel2=abs(vel2);
   digitalWrite(DIR_R,LOW);
   analogWrite(PWM_R,vel2);
   // Serial.println("Backward");
    }
   else if(vel2== 0) {  
     digitalWrite(PWM_R, vel2);
      }
  }
void leftEncoderISR()
{
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) { // BACKWORD DIRECTION
      leftCounter++;
    }else{
       leftCounter--;
    }
    } 
    else {
      if (digitalRead(LH_ENCODER_B) == LOW){
        leftCounter--;
      }else{
        leftCounter++;
      }
    }
  }

void rightEncoderISR()
{
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) ==LOW) {
      rightCounter--;
    }else{
       rightCounter++;
    }
    } 
    else {
      if (digitalRead(RH_ENCODER_B) == LOW){
      rightCounter++;
      }else{
        rightCounter--;
      }
    }
  }










ros::Subscriber<std_msgs::Float32> sub_1("OMOPLATE", &OMOPLATE_cb);
ros::Subscriber<std_msgs::Float32> sub_2("SHOULDER", &SHOULDER_cb);
ros::Subscriber<std_msgs::Float32> sub_3("ROTATE", &ROTATE_cb);
ros::Subscriber<std_msgs::Float32> sub_4("BICEPS", &BICEPS_cb);
ros::Subscriber<std_msgs::Float32> subl("lmotor_cmd", &lmotor_Cb );
ros::Subscriber<std_msgs::Float32> subr("rmotor_cmd", &rmotor_Cb );


//ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);


ros::Subscriber<std_msgs::Int16> inverse("Goal1_movebase", &inverse_);
ros::Subscriber<std_msgs::Int16> inverse1("Goal2_movebase", &inverse_1_);





void setup(){
  Serial.begin(57600);
 
  nh.initNode();
  nh.subscribe(sub_1);
  nh.subscribe(sub_2);
  nh.subscribe(sub_3);
  nh.subscribe(sub_4);
  nh.subscribe(subl);
  nh.subscribe(subr);
 // nh.subscribe(sub);

  nh.subscribe(inverse);
  nh.subscribe(inverse1);




  
  nh.advertise(p_OMOPLATE);
  nh.advertise(p_SHOULDER);
  nh.advertise(p_ROTATE);
  nh.advertise(p_BICEPS);
  nh.advertise (pub_Force);
  nh.advertise(pub_lencoder);
  nh.advertise(pub_rencoder);

  
  nh.advertise(pub_inverse);
  

  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  digitalWrite(RH_ENCODER_A, HIGH);      
  digitalWrite(RH_ENCODER_B, HIGH);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  digitalWrite(LH_ENCODER_A, HIGH);      
  digitalWrite(LH_ENCODER_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(21), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), leftEncoderISR,RISING);

  
  // OMOPLATE_L.attach(7);///
//   OMOPLATE_R.attach(16);///
 //  SHOULDER_L.attach(15);///
   // SHOULDER_R.attach(30);///
 //  ROTATE_L.attach(4);///
     ROTATE_R.attach(32);///
     BICEPS_L.attach(17);///
   BICEPS_R.attach(14);///

   delay(5000);

  OMOPLATE_L.write(75);  // 60 - 5
  OMOPLATE_R.write(123);  // 125 - 145
  
  SHOULDER_L.write(145);  //175-90
  SHOULDER_R.write(140);   //170 - 40 
  
  ROTATE_L.write(100); //110 - 130
  ROTATE_R.write(75);  // 60 - 95

  BICEPS_L.write(70);   //70 - 140
  BICEPS_R.write(120);  //120 - 40

  
   
//   PREPARE();
//   delay(5000);
//   CATCH();
//   delay(5000);
//   LIFT();
//   delay(10000);
//   RELEASE();

  
} 

void loop(){
  
  len_msg.data = leftCounter;
  ren_msg.data = rightCounter;


  
  int reading= analogRead (FSR_pin);
  Force_msg.data = reading ;
  pub_Force.publish(&Force_msg);
  
  pub_lencoder.publish( &len_msg );
  pub_rencoder.publish( &ren_msg );

   

  
  nh.spinOnce();
  //delay(1);
}



float radiansToDegrees(float position_radians)
{

  position_radians = position_radians;

  return position_radians * 57.2958;

}
