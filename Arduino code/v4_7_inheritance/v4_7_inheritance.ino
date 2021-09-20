 #include "MotorStepper.h"
#include "MotorManager.h"
#include "MotorDifferential.h"

#include <ros.h>

ros::NodeHandle  nh;

int minorStepsPerRev = 800; // written on the motor driver
int minorStepsPerMajorStep = minorStepsPerRev / 200; // how many minor steps are in between the 200 major steps of a standard stepper motor

double radPerMajorStep = PI / 100.0; // 2PI rads/200 steps
double radPerMinorStep = radPerMajorStep / minorStepsPerMajorStep;

//MotorStepper R1(13, 12, -45,  45, radPerMinorStep, 4.0);
//MotorStepper T1( 9, 8, -180, 180, radPerMinorStep, 4.0);
//MotorStepper T2(11, 10, -180, 180, radPerMinorStep, 4.0);
//MotorStepper R2( 7, 6, -180, 180, radPerMinorStep, 4.0);
MotorStepper R1(19,18, -45,  45, radPerMinorStep, 4.0);
MotorStepper T1(23,22,-180, 180, radPerMinorStep, 4.0);
MotorStepper T2(21,20,-180, 180, radPerMinorStep, 4.0);
MotorStepper R2(17,16,-180, 180, radPerMinorStep, 4.0);

// Arduino connect Rx-15 Tx-14
HardwareSerial& odrive_serial = Serial3;
MotorDifferential T3R3(odrive_serial, -90, 90, -180, 180, 0.005*2*PI, 3.95);

MotorManager mm(&R1, &T1, &T2, &R2, &T3R3);

void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(mm.sub);
  nh.advertise(mm.pub);

}

void loop() {
  // put your main code here, to run repeatedly:
  mm.checkSteps();
  nh.spinOnce();
}
