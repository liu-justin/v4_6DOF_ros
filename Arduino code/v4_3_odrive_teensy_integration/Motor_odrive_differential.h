#ifndef Motor_odrive_differential_h
#define Motor_odrive_differential_h

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

class Motor_odrive_differential {
  public:
    Motor_odrive_differential(HardwareSerial& odrive_serial);
    float vel_A;
    float vel_B;
    float pos_A;
    float pos_B;

    float vel_R3;
    float vel_T3;

    unsigned long previous_time_A;
    unsigned long previous_time_B;

    void update_motor_vel();
    void publish_motor_pos();
    void step();

    std_msgs::Float32 pos_R3_msg;
    std_msgs::Float32 pos_T3_msg;

    ros::Subscriber<std_msgs::Float32, Motor_odrive_differential> sub_R3;
    ros::Subscriber<std_msgs::Float32, Motor_odrive_differential> sub_T3;
    ros::Publisher pub_R3;
    ros::Publisher pub_T3;

    void message_R3_callback(const std_msgs::Float32& vel);
    void message_T3_callback(const std_msgs::Float32& vel);

    ODriveArduino odrive;
    
  private:
    float speed_ratio;
    float rad_to_rev;
    float final_multipler;

    float minor_steps;
};

#endif
