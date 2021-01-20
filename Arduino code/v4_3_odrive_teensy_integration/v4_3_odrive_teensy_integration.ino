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

//Motor_stepper R1( 2, 3, -45,  45, radPerMinorStep, 4.0, "vel_motorR1", "pos_motorR1");
//Motor_stepper T1( 6, 7,-180, 180, radPerMinorStep, 4.0, "vel_motorT1", "pos_motorT1");
//Motor_stepper T2( 4, 5,-180, 180, radPerMinorStep, 4.0, "vel_motorT2", "pos_motorT2");
//Motor_stepper R2(10,11,-180, 180, radPerMinorStep, 4.0, "vel_motorR2", "pos_motorR2");
Motor_stepper R1(19,18, -45,  45, radPerMinorStep, 4.0, "vel_motorR1", "pos_motorR1");
Motor_stepper T1(23,22,-180, 180, radPerMinorStep, 4.0, "vel_motorT1", "pos_motorT1");
Motor_stepper T2(21,20,-180, 180, radPerMinorStep, 4.0, "vel_motorT2", "pos_motorT2");
Motor_stepper R2(17,16,-180, 180, radPerMinorStep, 4.0, "vel_motorR2", "pos_motorR2");

// Arduino connect
HardwareSerial& odrive_serial = Serial3;

Motor_odrive_differential R3T3(odrive_serial);

void setup() {
  // this line doesn't work above the setup, dunno why
  nh.getHardware()->setBaud(115200);
  
  nh.initNode();
  nh.subscribe(R1.sub);
  nh.subscribe(T1.sub);
  nh.subscribe(T2.sub);
  nh.subscribe(R2.sub);
  nh.subscribe(R3T3.sub_R3);
  nh.subscribe(R3T3.sub_T3);
  nh.advertise(R1.pub);
  nh.advertise(T1.pub);
  nh.advertise(T2.pub);
  nh.advertise(R2.pub);
  nh.advertise(R3T3.pub_R3);
  nh.advertise(R3T3.pub_T3);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  R1.step();
  T1.step();
  T2.step();
  R2.step();
  R3T3.step();
  nh.spinOnce();
}
