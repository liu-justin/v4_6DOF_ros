#include <Motor.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;

int minorStepsPerRev = 800; // written on the motor driver
int minorStepsPerMajorStep = minorStepsPerRev/200; // how many minor steps are in between the 200 major steps of a standard stepper motor

double radPerMajorStep = PI/100.0; // 2PI rads/200 steps
double radPerMinorStep = radPerMajorStep/minorStepsPerMajorStep;

Motor R1(7,8,-45, 45, radPerMinorStep, "chatter_motorR1");
Motor T1(11,12,-180, 180, radPerMinorStep, "chatter_motorT1");
Motor T2(9,10,-180, 180, radPerMinorStep, "chatter_motorT2");
Motor R2(5,6, -180, 180, radPerMinorStep, "chatter_motorR2");

void messageCb_R1( const std_msgs::Int8& vel){
  R1.setVel(vel.data);
}
void messageCb_T1( const std_msgs::Int8& vel){
  T1.setVel(vel.data);
}
void messageCb_T2( const std_msgs::Int8& vel){
  T2.setVel(vel.data);
}
void messageCb_R2( const std_msgs::Int8& vel){
  R2.setVel(vel.data);
}

ros::Subscriber<std_msgs::Int8> sub1("chatter_motorR1", messageCb_R1 );
ros::Subscriber<std_msgs::Int8> sub2("chatter_motorT1", messageCb_T1 );
ros::Subscriber<std_msgs::Int8> sub3("chatter_motorT2", messageCb_T2 );
ros::Subscriber<std_msgs::Int8> sub4("chatter_motorR2", messageCb_R2 );

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
