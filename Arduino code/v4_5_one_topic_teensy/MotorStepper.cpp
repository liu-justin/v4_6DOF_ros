
#include "MotorStepper.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

// this value is being shared across all the instances of the class, need to fix
//std_msgs::Float32 posMsg;

MotorStepper::MotorStepper(int pulse, int direct, int lower, int upper, float stepSize, float multi)
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

  rads_per_step = stepSize;

}

float MotorStepper::getVel() {
  return vel;
}
float MotorStepper::getPos() {
  return pos;
}

void MotorStepper::setVel(float incoming_vel) {
  vel = multipler * incoming_vel;
  if (vel < 0) {
    digitalWrite(direction_pin, 0);
  }
  else if (vel > 0) {
    digitalWrite(direction_pin, 1);
  }
}

void MotorStepper::pulse() {
  digitalWrite(pulse_pin, HIGH);
  pulse_high = true;
  //  delayMicroseconds(5);
  //	digitalWrite(pulse_pin, LOW);
  // track the pos
  pos += ((vel > 0) - (vel < 0)) * rads_per_step / multipler;
}

// automatically finds correct time to pulse, based on inputed velocity
void MotorStepper::checkStep(unsigned long current_time) {
  // if the velocity is zero, then just skip, don't even count the time
  if (vel != 0) {
    if (pulse_high == true) {
      delayMicroseconds(3);
      digitalWrite(pulse_pin, LOW);
      pulse_high = false;
    }

    // equation is v = x/t --> t = x/v (if time period exceeds this, then...)
    if (((current_time - previous_time) / 1000.0) > (rads_per_step / float(abs(vel)))) {
      pulse();

      // reset previous_time for nextTime period
      previous_time = current_time;

    }
  }
}
