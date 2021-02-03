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

Motor_stepper R1(13,12, -45,  45, radPerMinorStep, 4.0);
Motor_stepper T1( 9, 8,-180, 180, radPerMinorStep, 4.0);
Motor_stepper T2(11,10,-180, 180, radPerMinorStep, 4.0);
Motor_stepper R2( 7, 6,-180, 180, radPerMinorStep, 4.0);
//Motor_stepper R1(29,30, -45,  45, radPerMinorStep, 4.0);
//Motor_stepper T1(25,26,-180, 180, radPerMinorStep, 4.0);
//Motor_stepper T2(27,28,-180, 180, radPerMinorStep, 4.0);
//Motor_stepper R2(31,32,-180, 180, radPerMinorStep, 4.0);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
}

void loop() {
  // put your main code here, to run repeatedly:
  R1.step();
  T1.step();
  T2.step();
  R2.step();
  nh.spinOnce();
}