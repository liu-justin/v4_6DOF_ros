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
		void setVel(float incoming_vel);
		void pulse();
		void checkStep(unsigned long current_time);
		
	private:
		
		int pulsePin;
		int directionPin;

		int lowerLimit;
		int upperLimit;

		float pos;
		float vel;

		float multipler;
		float rads_per_step;
    unsigned long previous_time;


};

#endif
