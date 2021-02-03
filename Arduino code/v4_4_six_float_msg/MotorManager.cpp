
#include "MotorManager.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include "Motor_stepper.h"

//MotorManager::MotorManager(Motor_stepper& R1, Motor_stepper& T1, Motor_stepper& T2, Motor_stepper& R2)
//MotorManager::MotorManager(Motor_stepper& R1)
//: motor_R1(R1)
//{
//
//}

//MotorManager::MotorManager()
//{
//  Motor_stepper R1(13,12, -45,  45, 1, 4.0);
//  Motor_stepper T1( 9, 8,-180, 180, 1, 4.0);
//  Motor_stepper T2(11,10,-180, 180, 1, 4.0);
//  Motor_stepper R2( 7, 6,-180, 180, 1, 4.0);
//  motor_list[0] = &R1;
//  motor_list[1] = &T1;
//  motor_list[2] = &T2;
//  motor_list[3] = &R2;
//}

//int MotorManager::getVel() {
//  return vel;
//}
//
//int MotorManager::getPos() {
//  return pos;
//}

//void MotorManager::setVel(float incomingVel) {
//  vel = multipler*incomingVel;
//  if (vel < 0){
//    digitalWrite(directionPin, 0);
//  }
//  else if (vel > 0){
//    digitalWrite(directionPin, 1);
//  }
//}
