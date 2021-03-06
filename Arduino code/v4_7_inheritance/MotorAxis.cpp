
#include "MotorAxis.h"

#include <ros.h>

// this value is being shared across all the instances of the class, need to fix
//std_msgs::Float32 posMsg;

MotorAxis::MotorAxis(int lower, int upper, float step_size, float multi)
  : vel_queue(sizeof(float), 5, FIFO)
  , gap_queue(sizeof(unsigned long), 5, FIFO)
{

  lower_limit = lower;
  upper_limit = upper;

  multipler = multi;

  pos = 0.0;
  vel = 0.0;
  gap = 0;
  gap_timer = 0;

  rads_per_step = step_size;

}

float MotorAxis::getPos() {
  return pos;
}

void MotorAxis::pushVelAndGap(float incoming_vel, uint32_t incoming_gap) {
  float new_vel = multipler * incoming_vel;
  vel_queue.push(&new_vel);
  gap_queue.push(&incoming_gap);
}

void MotorAxis::popVelAndGap() {
  vel_queue.pop(&vel);
  gap_queue.pop(&gap);
  gap_timer = 0;
}

void MotorAxis::checkTimeGap() {
  if (gap_timer > gap && !vel_queue.isEmpty()) {
    popVelAndGap();
  }
//  if (vel_queue.isEmpty() && vel != 0) {
//    vel = 0;
//  }
}
