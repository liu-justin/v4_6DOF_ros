#include "Motor_stepper.h"
#include "Motor_odrive_differential.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

int minorStepsPerRev = 1600; // written on the motor driver
int minorStepsPerMajorStep = minorStepsPerRev/200; // how many minor steps are in between the 200 major steps of a standard stepper motor

double radPerMajorStep = PI/100.0; // 2PI rads/200 steps
double radPerMinorStep = radPerMajorStep/minorStepsPerMajorStep;

//Motor_stepper R1( 7, 8, -45,  45, radPerMinorStep, 4.0, "vel_motorR1", "pos_motorR1");
//Motor_stepper T1(11,12,-180, 180, radPerMinorStep, 4.0, "vel_motorT1", "pos_motorT1");
//Motor_stepper T2( 9,10,-180, 180, radPerMinorStep, 4.0, "vel_motorT2", "pos_motorT2");
//Motor_stepper R2( 5, 6,-180, 180, radPerMinorStep, 4.0, "vel_motorR2", "pos_motorR2");
Motor_stepper R1(29,30, -45,  45, radPerMinorStep, 4.0, "vel_motorR1", "pos_motorR1");
Motor_stepper T1(25,26,-180, 180, radPerMinorStep, 4.0, "vel_motorT1", "pos_motorT1");
Motor_stepper T2(27,28,-180, 180, radPerMinorStep, 4.0, "vel_motorT2", "pos_motorT2");
Motor_stepper R2(31,32,-180, 180, radPerMinorStep, 4.0, "vel_motorR2", "pos_motorR2");

Motor_odrive_differential R3T3();
//float x = R3T3.final_multipler;

void setup() {
  // put your setup code here, to run once:
  odrive_serial.begin(115200);
  
  nh.initNode();
  nh.subscribe(R1.sub);
  nh.subscribe(T1.sub);
  nh.subscribe(T2.sub);
  nh.subscribe(R2.sub);
//  nh.subscribe(R3T3.sub_R3);
//  nh.subscribe(R3T3.sub_T3);
  nh.advertise(R1.pub);
  nh.advertise(T1.pub);
  nh.advertise(T2.pub);
  nh.advertise(R2.pub);
//  nh.advertise(R3T3.pub_R3);
//  nh.advertise(R3T3.pub_T3);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  R1.step();
  T1.step();
  T2.step();
  R2.step();
  nh.spinOnce();
}
