#ifndef MotorStepper_h
#define MotorStepper_h

/*provides stepper MotorStepper control thru single steps*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

class MotorStepper {
	public:
		MotorStepper(int pulse, int direct, int lower, int upper,float stepSize, float multipler);
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
