#ifndef MotorStepper_h
#define MotorStepper_h

/*provides stepper MotorStepper control thru single steps*/

#include <ros.h>
#include "MotorAxis.h"


class MotorStepper : public MotorAxis {
	public:
		MotorStepper(int pulse, int direct, int lower, int upper, float step_size, float multi);
		void pulse();
		void checkStep(unsigned long current_time);

	private:
		
		int pulse_pin;
		int direction_pin;

    float rads_per_step;

  	unsigned long previous_time;


};

#endif
