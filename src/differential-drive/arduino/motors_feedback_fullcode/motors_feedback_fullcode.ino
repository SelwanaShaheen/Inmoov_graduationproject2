#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
// left motor//////
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
volatile  long int rightCounter = 0; 
volatile  long int leftCounter = 0; 
/////////// Motors call back functions and sub_nodes ////////
ros::NodeHandle  nh1;
void lmotor_Cb( const std_msgs::Float32& lm){
  
  left_motor_control(lm.data);
}

void rmotor_Cb( const std_msgs::Float32& rm){
  
  right_motor_control(rm.data);
}

ros::Subscriber<std_msgs::Float32> subl("lmotor_cmd", &lmotor_Cb );
ros::Subscriber<std_msgs::Float32> subr("rmotor_cmd", &rmotor_Cb );


                              //////// Encoders functions and pub_nodes ////////

std_msgs::Int16 len_msg;
std_msgs::Int16 ren_msg;
ros::Publisher pub_lencoder( "lwheel", &len_msg);
ros::Publisher pub_rencoder( "rwheel", &ren_msg);
void setup()
{ 
  nh1.initNode();
  Serial.begin(57600);
  ////// motor////
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  nh1.subscribe(subl);
  nh1.subscribe(subr);
  /////////////////Encoders/////////
    //nh.initNode();
  nh1.advertise(pub_lencoder);
  nh1.advertise(pub_rencoder);
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

}
void loop()
{
    len_msg.data = leftCounter;
    ren_msg.data = rightCounter;
    
    pub_lencoder.publish( &len_msg );
    pub_rencoder.publish( &ren_msg );
    
    nh1.spinOnce();
 //right_motor_control(-100);
   //left_motor_control(-100);
  
    //Serial.println(leftCounter);
   //Serial.println(rightCounter);
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
