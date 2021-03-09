#ifndef MotorDifferential_h
#define MotorDifferential_h

#include <ros.h>

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#include "MotorAxis.h"

class MotorDifferential {
  public:
    MotorDifferential(HardwareSerial& odrive_serial, int tilt_lower, int tilt_upper, int rot_lower, int rot_upper, float step_size, float multipler);

    unsigned long previous_time_A;
    unsigned long previous_time_B;

    void checkStep(unsigned long current_time);

    void setVelTilt(float new_vel);
    void setVelRot(float new_vel);
    
    void checkTimeGap();

    void pushVelAndGapTilt(float incoming_vel, uint32_t incoming_gap);
    void pushVelAndGapRotation(float incoming_vel, uint32_t incoming_gap);


    ODriveArduino odrive;
    MotorAxis tilt;
    MotorAxis rotation;
    
  private:
    float speed_ratio;

    float vel_A;
    float vel_B;
    float pos_A;
    float pos_B;

    float rad_per_step;

};

#endif
