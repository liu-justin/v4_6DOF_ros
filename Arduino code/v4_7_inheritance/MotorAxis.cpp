
#include "MotorAxis.h"

#include <ros.h>

// velocity value in queue is the transformed input velocity needed to generate the output velocity

MotorAxis::MotorAxis(int lower, int upper)
  : vel_queue(sizeof(float), 5, FIFO)
  , gap_queue(sizeof(unsigned long), 5, FIFO)
{

  lower_limit = lower;
  upper_limit = upper;

  pos = 0.0;
  vel = 0.0;
  gap = 0;
  gap_timer = 0;

}

float MotorAxis::getPos() {
  return pos;
}

float MotorAxis::getVel() {
  return vel;
}

void MotorAxis::pushVelAndGap(float incoming_vel, uint32_t incoming_gap) {
  float new_vel = incoming_vel;
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
