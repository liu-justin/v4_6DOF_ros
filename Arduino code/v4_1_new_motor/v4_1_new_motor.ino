#include <Motor_stepper.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

int minorStepsPerRev = 1600; // written on the motor driver
int minorStepsPerMajorStep = minorStepsPerRev/200; // how many minor steps are in between the 200 major steps of a standard stepper motor

double radPerMajorStep = PI/100.0; // 2PI rads/200 steps
double radPerMinorStep = radPerMajorStep/minorStepsPerMajorStep;

Motor_stepper R1( 8, 9, -45,  45, radPerMinorStep, 4.0, "chatter_motorR1");
Motor_stepper T1(12,13,-180, 180, radPerMinorStep, 4.0, "chatter_motorT1");
Motor_stepper T2(10,11,-180, 180, radPerMinorStep, 4.0, "chatter_motorT2");
Motor_stepper R2( 6, 7,-180, 180, radPerMinorStep, 4.0, "chatter_motorR2");
//Motor_stepper R1(33,35, -45,  45, radPerMinorStep, 4.0, "chatter_motorR1");
//Motor_stepper T1(32,31,-180, 180, radPerMinorStep, 4.0, "chatter_motorT1");
//Motor_stepper T2(30,29,-180, 180, radPerMinorStep, 4.0, "chatter_motorT2");
//Motor_stepper R2(28,27,-180, 180, radPerMinorStep, 4.0, "chatter_motorR2");

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

ros::Subscriber<std_msgs::Float32> sub1("chatter_motorR1", messageCb_R1 );
ros::Subscriber<std_msgs::Float32> sub2("chatter_motorT1", messageCb_T1 );
ros::Subscriber<std_msgs::Float32> sub3("chatter_motorT2", messageCb_T2 );
ros::Subscriber<std_msgs::Float32> sub4("chatter_motorR2", messageCb_R2 );

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
