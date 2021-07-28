#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <SoftwareSerial.h> // TX RX software library for bluetooth

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
   delay(2000);
   CATCH();
   delay(2000);
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
   delay(2000);
   RELEASE();
   sub = 0 ;
  }
}



void PREPARE (){
  float biceps_L = map(0, 0, 90, 70, 150);
  float biceps_R = map(0, 0, 90, 115, 40);
  float rotate_L = map(0, 0, 90, 110, 130);
  float rotate_R = map(0, 0, 90, 75, 95);
  float shoulder_L = map( 45, 0, 90, 175,90);
  float shoulder_R = map(45, 0, 90, 170,40);
  float omoplate_L = map(45 , 0, 90,60, 5);
  float omoplate_R = map(45, 0, 90,163,135);
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R); 
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
}


void CATCH (){
  float biceps_L = map(30, 0, 90, 70, 150);
  float biceps_R = map(30, 0, 90, 115, 40);
  float rotate_L = map(50, 0, 90, 110, 130);
  float rotate_R = map(50, 0, 90, 75, 95);
  float shoulder_L = map( 45 , 0, 90, 175,90);
  float shoulder_R = map(45, 0, 90, 170,40);
  float omoplate_L = map(0 , 0, 90,60, 5);
  float omoplate_R = map(0, 0, 90,161,135);
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R); 
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
}


void LIFT (){
  float biceps_L = map(30, 0, 90, 70, 150);
  float biceps_R = map(30, 0, 90, 115, 40);
  float rotate_L = map(50, 0, 90, 110, 130);
  float rotate_R = map(50, 0, 90, 75, 95);
  float shoulder_L = map( 70 , 0, 90, 175,90);
  float shoulder_R = map(70, 0, 90, 170,40);
  float omoplate_L = map(0 , 0, 90,60, 5);
  float omoplate_R = map(0, 0, 90,161,135);
  OMOPLATE_L.write(omoplate_L); //set servo angle, should be from 0-180  
  OMOPLATE_R.write(omoplate_R);
  BICEPS_L.write(biceps_L); //set servo angle, should be from 0-180  
  BICEPS_R.write(biceps_R);
  SHOULDER_L.write(shoulder_L); //set servo angle, should be from 0-180
  SHOULDER_R.write(shoulder_R); 
  ROTATE_L.write(rotate_L); //set servo angle, should be from 0-180  
  ROTATE_R.write(rotate_R);
}


void RELEASE (){
  float biceps_L = map(0, 0, 90, 70, 150);
  float biceps_R = map(0, 0, 90, 115, 40);
  float rotate_L = map(0, 0, 90, 110, 130);
  float rotate_R = map(0, 0, 90, 75,95);
  float shoulder_L = map( 0 , 0, 90, 175,90);
  float shoulder_R = map(0, 0, 90, 170,40);
  float omoplate_L = map(0 , 0, 90,60, 5);
  float omoplate_R = map(0, 0, 90,163,135);
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
  float omoplate_R = map(OMOPLATE, 0, 90,160,135);
  Serial.print(OMOPLATE); 

  float SHOULDER = radiansToDegrees(cmd_msg.position[5]);
  float shoulder_L = map(SHOULDER, 0, 90, 175,90);
  float shoulder_R = map(SHOULDER, 0, 90, 170,40);

  float ROTATE = radiansToDegrees(cmd_msg.position[6]);
  float rotate_L = map(ROTATE, 0, 90, 110, 130);
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
/////////////////////////////////////////////////////////////////



#define bluetoothTx  18 // bluetooth tx to 10 pin
#define bluetoothRx  19 // bluetooth rx to 11 pin
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

/////////////////////////////////////////////////////////////

int  incoming_value;
int flag=0;        //makes sure that the serial only prints once the state
int stateStop=0;




void setup(){
  Serial.begin(9600);  //Setup usb serial connection to computer
  bluetooth.begin(9600);    //Set
  nh.getHardware()->setBaud(115200);
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

  
   OMOPLATE_L.attach(7);/////////////////////////
   OMOPLATE_R.attach(16);///
   SHOULDER_L.attach(15);///
   SHOULDER_R.attach(30);///
   ROTATE_L.attach(4);///
   ROTATE_R.attach(32);///
   BICEPS_L.attach(17);///
   BICEPS_R.attach(14);///

  OMOPLATE_L.write(65);  // 60 - 5
  OMOPLATE_R.write(160);  // 163 - 135
  
  SHOULDER_L.write(175);  //175-90
  SHOULDER_R.write(170);   //170 - 40 
  
  ROTATE_L.write(110); //110 - 130
  ROTATE_R.write(75);  // 60 - 95

  BICEPS_L.write(70);   //70 - 150
  BICEPS_R.write(115);  //120 - 40
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


   //Read from bluetooth and write to usb serial
  if(bluetooth.available()>=2)
  {
    unsigned int servopos = bluetooth.read();
    Serial.println(servopos);
    flag=0;
    unsigned int servopos1 = bluetooth.read();
    Serial.println(servopos1);
    unsigned int realservo = (servopos1 *256) + servopos;
    Serial.println(realservo);
//    unsigned int realservo2 = (servopos1 *265) + servopos;
//    Serial.println(realservo2);


//###########################OMOPLATE#####################//
   if (realservo >= 1000 && realservo <1050  ) {
      int omoplateL = realservo;
      int omoplateR = realservo;
      omoplateL = map(omoplateL, 1000, 1050,65, 5);
      omoplateR = map(omoplateR, 1000, 1050,163,135);
      OMOPLATE_L.write(omoplateL);
      OMOPLATE_R.write(omoplateR);
      Serial.println(omoplateL);
      Serial.println(omoplateR);
      Serial.println("OMOPLATE IS ON");
      delay(10);
    }
//###########################SHOULDER#####################//
    if (realservo >= 2000 && realservo <2045) {
      int shoulderL = realservo;
      int shoulderR = realservo;
      shoulderL = map(shoulderL, 2000, 2045, 175,90);
      shoulderR = map(shoulderR, 2000, 2045, 170, 80);//170-40
      SHOULDER_L.write(shoulderL);
      Serial.println(shoulderL);
      SHOULDER_R.write(shoulderR);
      Serial.println(shoulderR);
      Serial.println("SHOULDER IS ON");
      delay(10);
    }
//###########################ROTATE#####################//
    if (realservo >= 8000 && realservo <8060) {
      int rotateL = realservo;
      int rotateR = realservo;
      rotateL = map(rotateL, 8000, 8060, 110, 130  );
      rotateR = map(rotateR, 8000, 8060, 75,95);
      ROTATE_L.write(rotateL);
      ROTATE_R.write(rotateR);
      Serial.println(rotateL);
      Serial.println(rotateR);
      Serial.println("ROTATE IS ON");
      delay(10);
    }
//###########################BICEPS#####################// 
    if (realservo >= 9000 && realservo <9090) {
      int bicepsL = realservo;
      int bicepsR = realservo;
      bicepsL = map(bicepsL, 9000, 9090, 70, 150);
      bicepsR = map(bicepsR, 9000, 9090,  115, 40);
      BICEPS_L.write(bicepsL);
      BICEPS_R.write(bicepsR);
      Serial.println("BICEPS IS ON");
      delay(10);
    }
////###########################REST#####################// 
//    if (realservo >= 5000 && realservo <5090) {
//      int restL = realservo;
//      int restR = realservo;
//      restL = map(restL, 5000, 5090, 130, 0);
//      restR = map(restR, 5000, 5090, 0, 90);
//      REST_L.write(restL);
//      REST_R.write(restR);
//      Serial.println("REST IS ON");
//      delay(10);
//    }
//###########################FINGERS#####################// 
//    if (realservo >= 6000 && realservo <6090) {
//      int fingers1R = realservo;
//      int fingers2R = realservo;
//      int fingers3R = realservo;
//      int fingers4R = realservo;
//      int fingers5R = realservo;
//      int fingers1L = realservo;
//      int fingers2L = realservo;
//      int fingers3L = realservo;
//      int fingers4L = realservo;
//      int fingers5L = realservo;
//      //fingers1R = map(fingers1R, 6000, 6090, 0, 90);
//      fingers2R = map(fingers2R, 6000, 6090, 0, 90);
//      fingers3R = map(fingers3R, 6000, 6090, 10, 90);
//      fingers4R = map(fingers4R, 6000, 6090, 0, 90);
//      //fingers5R = map(fingers5R, 6000, 6090, 0, 90);
//      fingers1L = map(fingers1L, 6000, 6090, 0, 90);
//      //fingers2L = map(fingers2L, 6000, 6090, 0, 90);
//      //fingers3L = map(fingers3L, 6000, 6090, 0, 90);
//      fingers4L = map(fingers4L, 6000, 6090, 0, 90);
//      fingers5L = map(fingers5L, 6000, 6090, 0, 90);
//      FINGER_1L.write(fingers1L);
//      FINGER_2L.write(fingers2L);
//      FINGER_3L.write(fingers3L);
//      FINGER_4L.write(fingers4L);
//      FINGER_5L.write(fingers5L);
//      FINGER_1R.write(fingers1R);
//      FINGER_2R.write(fingers2R);
//      FINGER_3R.write(fingers3R);
//      FINGER_4R.write(fingers4R);
//      FINGER_5R.write(fingers5R);
//      Serial.println("FINGERS IS ON");
//      delay(10);
//    }
//////###########################TWIST#####################//
//    if (servopos == 20){TWIST.write(0);}
//    else if (servopos == 21){TWIST.write(45);}
//    else if (servopos == 22){TWIST.write(90);}
//    else if (servopos == 23){TWIST.write(135);}
//    else if (servopos == 24){TWIST.write(180);}
//////###########################UP&DOWN#####################//
//    if (servopos == 30){UP_DOWN.write(60);}
//    else if (servopos == 31){UP_DOWN.write(80);}
//    else if (servopos == 32){UP_DOWN.write(110);}
//    else if (servopos == 33){UP_DOWN.write(135);}
//    else if (servopos == 34){UP_DOWN.write(180);}  
//////###########################EYES#####################//
//    if (servopos == 40){EYES.write(70);}
//    else if (servopos == 41){EYES.write(90);}
//    else if (servopos == 42){EYES.write(120);}
//////###########################MOUSE#####################//
//    if (servopos == 50){MOUSE.write(0);Serial.println(servopos);}
//    else if (servopos == 51){MOUSE.write(180);Serial.println(servopos);}
/////////////////////stop///////////////////////////////
if (servopos == 'S' || stateStop == 1)
  {
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L,0);
    if(flag == 0){
          Serial.println("STOP!");
          flag=1;}
  }
//////////////////Move Backword////////////////////////////////////
  
   if (servopos == 'B')
  {
    digitalWrite(DIR_R,LOW);
    analogWrite(PWM_R,100);
    //digitalWrite(DIR1_R, LOW);
    
    digitalWrite(DIR_L,LOW);
     analogWrite(PWM_L,106);
    //digitalWrite(DIR1_L, LOW);
    if(flag == 0){
    Serial.println("Reverse!");
    flag=1;}
  }
/////////////////Move Forward///////////////////////////
   if (servopos == 'F')
  {
     digitalWrite(DIR_R,HIGH);
     analogWrite(PWM_R,100);
   // digitalWrite(DIR2_R,LOW);
    
    digitalWrite(DIR_L,HIGH);
    analogWrite(PWM_L,106);
    //digitalWrite(DIR2_L, LOW);
    if(flag == 0){
    Serial.println("Go Forward!");
    flag=1;}

  }
////////////////Spin left//////////////////
   if (servopos == 'L' )
  {
    digitalWrite(DIR_R,HIGH);
    analogWrite(PWM_R,50);
   // digitalWrite(DIR2_R,LOW);
    
    digitalWrite(DIR_L,LOW);
    analogWrite(PWM_L,53);
    //digitalWrite(DIR1_L, LOW);
    if(flag == 0){
    Serial.println("Turn LEFT");
    flag=1;}
   }
//////////////////Spin right/////////////////
   if (servopos == 'R')
  { 
    digitalWrite(DIR_R,LOW);
    analogWrite(PWM_R,50);
   // digitalWrite(DIR2_R,LOW);
    
    digitalWrite(DIR_L,HIGH);
    analogWrite(PWM_L,53);
    //digitalWrite(DIR2_L, LOW);
    if(flag == 0){
    Serial.println("Turn RIGHT");
    flag=1;}

  }
  
  }
   nh.spinOnce();
}



float radiansToDegrees(float position_radians)
{

  position_radians = position_radians;

  return position_radians * 57.2958;

}
