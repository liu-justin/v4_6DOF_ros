#include "Arduino.h"
#include "Motor.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

Motor::Motor(int pulse, int direct, int lower, int upper, float stepSize, String chatter){
	pulsePin = pulse;
	directionPin = direct;
	pinMode(pulsePin, OUTPUT);
	pinMode(directionPin, OUTPUT);

	lowerLimit = lower;
	upperLimit = upper;

	pos = 0;
	vel = 0;

	radiansPerMinorStep = stepSize;
}

int Motor::getVel() {
	return vel;
}

int Motor::getPos() {
	return pos;
}

void Motor::setVel(int incomingVel) {
	vel = incomingVel;
	if (vel < 0){
		digitalWrite(directionPin, 0);
	}
	else if (vel > 0){
		digitalWrite(directionPin, 1);
	}
}

void Motor::pulse(){
	digitalWrite(pulsePin, HIGH);
	digitalWrite(pulsePin, LOW);
	// track the pos
	pos += (vel > 0) - (vel < 0);
}

// automatically finds correct time to pulse, based on inputed velocity
void Motor::step(){
	// if the velocity is zero, then just skip, don't even count the time
	if (vel != 0){

		// get time for time period measurement
		unsigned long timeA = millis();

		// equation is v = x/t --> t = x/v (if time period exceeds this, then...)
		// was just time-previousTime before, forgot it output milliseconds, needed to divide by 1000 to get secs
		if (((timeA-previousTime)/1000.0) > (radiansPerMinorStep/float(abs(vel)))){   
			pulse();         

			// reset previousTime for nextTime period                 
			previousTime = millis();          
		}
	}
}

// put aside for now, was for when the subscriber was in the class
// but i couldn't figure out how to put sub in class
void Motor::messageCb( const std_msgs::Int8& vel){
  if (vel.data == 0) {
    digitalWrite(2, LOW);
  }
  else {
    digitalWrite(2, HIGH); 
  }
}

