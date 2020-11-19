#ifndef Motor_h
#define Motor_h

/*provides stepper Motor control thru single steps*/

#include "Arduino.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

class Motor {
	public:
		Motor(int pulse, int direct, int lower, int upper,float stepSize, String chatter);
		int getPos();
		int getVel();
		void setVel(int incomingVel);
		void pulse();
		void step();

		unsigned long previousTime;

		void messageCb( const std_msgs::Int8& vel);
		
		// Motor(int pulse, int direct, int lower, int upper, String chatter):sub("chatter", &Motor::messageCb);
		
		
	private:
		
		int pulsePin;
		int directionPin;

		int lowerLimit;
		int upperLimit;

		int pos;
		int vel;

		float radiansPerMinorStep;


};

#endif