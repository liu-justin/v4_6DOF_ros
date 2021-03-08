
#include "MotorStepper.h"

#include <ros.h>

// this value is being shared across all the instances of the class, need to fix
//std_msgs::Float32 posMsg;

MotorStepper::MotorStepper(int pulse, int direct, int lower, int upper, float step_size, float multi)
: MotorStepper::MotorAxis(lower, upper)
{
  pulse_pin = pulse;
  direction_pin = direct;
  pinMode(pulse_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);

  speed_ratio = multi;
  rads_per_step = step_size;
  pulse_high = false;

}

void MotorStepper::pulse() {
  digitalWrite(pulse_pin, HIGH);
  digitalWrite(direction_pin, 1 - (vel < 0));
  pulse_high = true;
  
  // track the pos
  pos += ((vel > 0) - (vel < 0)) * rads_per_step;
}

// automatically finds correct time to pulse, based on inputed velocity
void MotorStepper::checkStep(unsigned long current_time) {
  checkTimeGap();
  
  // allowing time for driver to register the high, then sets pulse to low
  if (pulse_high == true && current_time - previous_time >= 10) {
    digitalWrite(pulse_pin, LOW);
    pulse_high = false;
  }

  // main step check
  if (vel != 0) {
    // equation is v = x/t --> t = x/v (if time period exceeds this, then...)
    if (((current_time - previous_time) / 1000000.0) > (rads_per_step / float(abs(speed_ratio*vel)))) {
      pulse();

      // reset previous_time for nextTime period
      previous_time = current_time;

    }
  }
}
