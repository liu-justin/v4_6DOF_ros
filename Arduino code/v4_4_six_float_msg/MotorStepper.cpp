
#include "MotorStepper.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

// this value is being shared across all the instances of the class, need to fix
//std_msgs::Float32 posMsg;

MotorStepper::MotorStepper(int pulse, int direct, int lower, int upper, float stepSize, float multi)
{
	pulsePin = pulse;
	directionPin = direct;
	pinMode(pulsePin, OUTPUT);
	pinMode(directionPin, OUTPUT);

	lowerLimit = lower;
	upperLimit = upper;

	multipler = multi;

	pos = 0.0;
	vel = 0.0;

	rads_per_step = stepSize;

 }

int MotorStepper::getVel() {
	return vel;
}

int MotorStepper::getPos() {
	return pos;
}

void MotorStepper::setVel(float incoming_vel) {
	vel = multipler*incoming_vel;
	if (vel < 0){
		digitalWrite(directionPin, 0);
	}
	else if (vel > 0){
		digitalWrite(directionPin, 1);
	}
}

void MotorStepper::pulse(){
	digitalWrite(pulsePin, HIGH);
	digitalWrite(pulsePin, LOW);
	// track the pos
	pos += ((vel > 0) - (vel < 0))*rads_per_step/multipler;
}

// automatically finds correct time to pulse, based on inputed velocity
void MotorStepper::checkStep(unsigned long current_time){
	// if the velocity is zero, then just skip, don't even count the time
	if (vel != 0){

		// equation is v = x/t --> t = x/v (if time period exceeds this, then...)
		if (((current_time-previous_time)/1000.0) > (rads_per_step/float(abs(vel)))){   
			pulse();         

			// reset previous_time for nextTime period                 
			previous_time = current_time;

		}
	}
}
