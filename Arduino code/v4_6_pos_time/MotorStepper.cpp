
#include "MotorStepper.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

// this value is being shared across all the instances of the class, need to fix
//std_msgs::Float32 posMsg;

MotorStepper::MotorStepper(int pulse, int direct, int lower, int upper, float stepSize, float multi)
  : vel_queue(sizeof(float), 5, FIFO)
  , gap_queue(sizeof(unsigned long), 5, FIFO)
{
  pulse_pin = pulse;
  direction_pin = direct;
  pinMode(pulse_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);

  lowerLimit = lower;
  upperLimit = upper;

  multipler = multi;

  pos = 0.0;
  vel = 0.0;
  gap = 0;
  gap_timer = 0;

  rads_per_step = stepSize;

}

float MotorStepper::getPos() {
  return pos;
}

void MotorStepper::pushVelAndGap(float incoming_vel, unsigned long incoming_gap) {
  float new_vel = multipler * incoming_vel;
  vel_queue.push(&new_vel);
  gap_queue.push(&incoming_gap);
}

void MotorStepper::popVel() {
  vel_queue.pop(&vel);
  if (vel < 0) {
    digitalWrite(direction_pin, 0);
  }
  else if (vel > 0) {
    digitalWrite(direction_pin, 1);
  }
  gap_queue.pop(&gap);
  gap_timer = 0;
}

void MotorStepper::pulse() {
  digitalWrite(pulse_pin, HIGH);
  pulse_high = true;
  // track the pos
  pos += ((vel > 0) - (vel < 0)) * rads_per_step / multipler;
}

void MotorStepper::checkTimeGap() {
  if (gap_timer > gap && !gap_queue.isEmpty()){
    popVel();
  }
}

// automatically finds correct time to pulse, based on inputed velocity
void MotorStepper::checkStep(unsigned long current_time) {
  // allowing time for driver to register the high, then sets pulse to low
  if (pulse_high == true && current_time - previous_time >= 10) {
    digitalWrite(pulse_pin, LOW);
    pulse_high = false;
  }
  if (vel != 0) {
    // equation is v = x/t --> t = x/v (if time period exceeds this, then...)
    if (((current_time - previous_time) / 1000000.0) > (rads_per_step / float(abs(vel)))) {
      pulse();

      // reset previous_time for nextTime period
      previous_time = current_time;

    }
  }
}
