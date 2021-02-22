#ifndef MotorStepper_h
#define MotorStepper_h

/*provides stepper MotorStepper control thru single steps*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <cppQueue.h>


class MotorStepper {
	public:
		MotorStepper(int pulse, int direct, int lower, int upper,float stepSize, float multipler);
		float getPos();
		float getVel();
		void setVel(float incoming_vel);
    void pushVelAndGap(float incoming_vel, unsigned long incoming_gap);
    void popVel();
		void pulse();
		void checkStep(unsigned long current_time);
    void checkTimeGap();
		
	private:
		
		int pulse_pin;
		int direction_pin;

		int lowerLimit;
		int upperLimit;

		float pos;
    float vel;
    float gap;
    cppQueue vel_queue;
    elapsedMicros gap_timer;
    cppQueue gap_queue;

		float multipler;
		float rads_per_step;
    unsigned long previous_time;

    bool pulse_high;


};

#endif
