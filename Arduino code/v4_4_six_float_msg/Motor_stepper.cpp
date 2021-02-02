
#include "Motor_stepper.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

// this value is being shared across all the instances of the class, need to fix
//std_msgs::Float32 posMsg;

Motor_stepper::Motor_stepper(int pulse, int direct, int lower, int upper, float stepSize, float multi)
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

	radiansPerMinorStep = stepSize;

 }

int Motor_stepper::getVel() {
	return vel;
}

int Motor_stepper::getPos() {
	return pos;
}

void Motor_stepper::setVel(float incomingVel) {
	vel = multipler*incomingVel;
	if (vel < 0){
		digitalWrite(directionPin, 0);
	}
	else if (vel > 0){
		digitalWrite(directionPin, 1);
	}
}

void Motor_stepper::pulse(){
	digitalWrite(pulsePin, HIGH);
	digitalWrite(pulsePin, LOW);
	// track the pos
	pos += ((vel > 0) - (vel < 0))*radiansPerMinorStep/multipler;
}

// automatically finds correct time to pulse, based on inputed velocity
void Motor_stepper::step(){
	// if the velocity is zero, then just skip, don't even count the time
	if (vel != 0){

		// get time for time period measurement
		unsigned long timeA = millis();

		// equation is v = x/t --> t = x/v (if time period exceeds this, then...)
		if (((timeA-previousTime)/1000.0) > (radiansPerMinorStep/float(abs(vel)))){   
			pulse();         

			// reset previousTime for nextTime period                 
			previousTime = millis();

		}
	}
}
