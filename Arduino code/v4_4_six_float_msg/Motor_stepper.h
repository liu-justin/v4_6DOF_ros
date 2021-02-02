#ifndef Motor_stepper_h
#define Motor_stepper_h

/*provides stepper Motor_stepper control thru single steps*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

class Motor_stepper {
	public:
		Motor_stepper(int pulse, int direct, int lower, int upper,float stepSize, float multipler);
		int getPos();
		int getVel();
		void setVel(float incomingVel);
		void pulse();
		void step();

		unsigned long previousTime;

		std_msgs::Float32 posMsg;
		
	private:
		
		int pulsePin;
		int directionPin;

		int lowerLimit;
		int upperLimit;

		float pos;
		float vel;

		float multipler;

		float radiansPerMinorStep;


};

#endif
