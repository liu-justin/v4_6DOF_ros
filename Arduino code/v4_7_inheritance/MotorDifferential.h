#ifndef MotorDifferential_h
#define MotorDifferential_h

#include <ros.h>

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#include <cppQueue.h>

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

    void pushVelandGapT3(float incoming_vel, unsigned long incoming_gap);
    void pushVelandGapR3(float incoming_vel, unsigned long incoming_gap);
    void popVelandGapT3();
    void popVelandGapR3();
    void checkTimeGap();


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

//    cppQueue vel_queue_T3;
//    cppQueue gap_queue_T3;
//    elapsedMicros gap_timer_T3;
//      uint32_t gap_T3;
//
//    cppQueue vel_queue_R3;
//    cppQueue gap_queue_R3;
//    elapsedMicros gap_timer_R3;
//     uint32_t gap_R3;
};

#endif
