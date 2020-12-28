#include "Motor_stepper.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

int minorStepsPerRev = 1600; // written on the motor driver
int minorStepsPerMajorStep = minorStepsPerRev/200; // how many minor steps are in between the 200 major steps of a standard stepper motor

double radPerMajorStep = PI/100.0; // 2PI rads/200 steps
double radPerMinorStep = radPerMajorStep/minorStepsPerMajorStep;

Motor_stepper R1( 7, 8, -45,  45, radPerMinorStep, 4.0, 'vel_motorR1');
Motor_stepper T1(11,12,-180, 180, radPerMinorStep, 4.0, 'vel_motorT1');
Motor_stepper T2( 9,10,-180, 180, radPerMinorStep, 4.0, 'vel_motorT2');
Motor_stepper R2( 5, 6,-180, 180, radPerMinorStep, 4.0, 'vel_motorR2');

void messageCb_R1( const std_msgs::Float32& vel){
  R1.setVel(vel.data);
}
void messageCb_T1( const std_msgs::Float32& vel){
  T1.setVel(vel.data);
}
void messageCb_T2( const std_msgs::Float32& vel){
  T2.setVel(vel.data);
}
void messageCb_R2( const std_msgs::Float32& vel){
  R2.setVel(vel.data);
}

//std_msgs::Float32 float32;

ros::Subscriber<std_msgs::Float32> sub1("vel_motorR1", messageCb_R1 );
ros::Subscriber<std_msgs::Float32> sub2("vel_motorT1", messageCb_T1 );
ros::Subscriber<std_msgs::Float32> sub3("vel_motorT2", messageCb_T2 );
ros::Subscriber<std_msgs::Float32> sub4("vel_motorR2", messageCb_R2 );
//ros::Publisher pubR1("pos_motorR1", &float32);
//ros::Publisher pubT1("pos_motorT1", &float32);
//ros::Publisher pubR2("pos_motorR2", &float32);
//ros::Publisher pubT2("pos_motorT2", &float32);


void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4); 
  
}

void loop() {
  // put your main code here, to run repeatedly:
  R1.step();
  T1.step();
  T2.step();
  R2.step();
  nh.spinOnce();
}
