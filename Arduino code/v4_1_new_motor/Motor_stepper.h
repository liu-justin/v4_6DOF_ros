#ifndef Motor_stepper_h
#define Motor_stepper_h

/*provides stepper Motor_stepper control thru single steps*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

class Motor_stepper {
	public:
		Motor_stepper(int pulse, int direct, int lower, int upper,float stepSize, float multipler, char velChatter[]);
		int getPos();
		int getVel();
		void setVel(float incomingVel);
		void pulse();
		void step();

		unsigned long previousTime;

		void messageCb( const std_msgs::Float32& vel);
		
//    ros::Subscriber<std_msgs::Float32, Motor_stepper> sub;
//    ros::Publisher pub;
		
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
