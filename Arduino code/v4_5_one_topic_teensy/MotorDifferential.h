#ifndef MotorDifferential_h
#define MotorDifferential_h

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

class MotorDifferential {
  public:
    MotorDifferential(HardwareSerial& odrive_serial);

    unsigned long previous_time_A;
    unsigned long previous_time_B;

    void updateVelAB();
    void publish_motor_pos();
    void checkStep(unsigned long current_time);

    void setVelT3(float new_vel);
    void setVelR3(float new_vel);

    float getPosT3();
    float getPosR3();

    ODriveArduino odrive;
    
  private:
    float speed_ratio;
    float rad_to_rev;
    float final_multipler;

    float vel_A;
    float vel_B;
    float pos_A;
    float pos_B;
    float vel_R3;
    float vel_T3;

    float minor_steps;
};

#endif
