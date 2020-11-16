#include "Arduino.h"
#include "Motor.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

// Motor::Motor(int pulse, int direct, int lower, int upper, String chatter){
	

// }

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
	pos += (vel > 0) - (vel < 0);
}

void Motor::step(int angle){
	if (vel != 0){
		unsigned long time = millis();
		if (time-previousTime > angle/vel){
			pulse();
		}
		previousTime = millis();
	}

}

void Motor::messageCb( const std_msgs::Int8& vel){
  if (vel.data == 0) {
    digitalWrite(2, LOW);
  }
  else {
    digitalWrite(2, HIGH); 
  }
}

