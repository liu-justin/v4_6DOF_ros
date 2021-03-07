#ifndef MotorAxis_h
#define MotorAxis_h

/*provides stepper MotorAxis control thru single steps*/

#include <ros.h>
#include <cppQueue.h>


class MotorAxis {
  public:
    MotorAxis(int lower, int upper, float multipler);
    float getPos();
    float getVel();
    void setVel(float incoming_vel);
    void pushVelAndGap(float incoming_vel, unsigned long incoming_gap);
    void popVelAndGap();
    void checkTimeGap();

    float getMinorSteps();
    

  protected:

    int lower_limit;
    int upper_limit;

    float pos;
    float vel;
    uint32_t gap; // gap between queue pops
    cppQueue vel_queue;
    cppQueue gap_queue;
    elapsedMicros gap_timer;


    float multipler; // torque ratio from output to input
    float rads_per_step;

    bool pulse_high;


};

#endif
