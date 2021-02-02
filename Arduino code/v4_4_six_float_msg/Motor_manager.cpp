
#include "Motor_manager.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include "Motor_stepper.h"

// this value is being shared across all the instances of the class, need to fix
//std_msgs::Float32 posMsg;

//Motor_manager::Motor_manager(&Motor_stepper R1, &Motor_stepper T1, &Motor_stepper T2, &Motor_stepper R2)
//{
//  motor_list[0] = R1;
//  motor_list[1] = T1;
//  motor_list[2] = T2;
//  motor_list[3] = R2;
//
//}

Motor_manager::Motor_manager()
{
  
}

//int Motor_manager::getVel() {
//  return vel;
//}
//
//int Motor_manager::getPos() {
//  return pos;
//}

//void Motor_manager::setVel(float incomingVel) {
//  vel = multipler*incomingVel;
//  if (vel < 0){
//    digitalWrite(directionPin, 0);
//  }
//  else if (vel > 0){
//    digitalWrite(directionPin, 1);
//  }
//}
