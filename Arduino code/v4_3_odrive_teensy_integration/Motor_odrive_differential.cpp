#include "Motor_odrive_differential.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Motor_odrive_differential::Motor_odrive_differential(SoftwareSerial& odrive_serial)
:sub_R3("vel_motorR3", &Motor_odrive_differential::message_R3_callback, this)
,sub_T3("vel_motorT3", &Motor_odrive_differential::message_T3_callback, this)
,pub_R3("pos_motorR3", &pos_R3_msg)
,pub_T3("pos_motorT3", &pos_T3_msg)
,odrive(odrive_serial)
{  
  vel_A = 0.0;
  vel_B = 0.0;
  pos_A = 0.0;
  pos_B = 0.0;
  vel_R3 = 0.0;
  vel_T3 = 0.0;
  previous_time_A = 0.0;
  previous_time_B = 0.0;

  speed_ratio = 3.95;
  rad_to_rev = 1/(2*PI);
  final_multipler = speed_ratio*rad_to_rev;
  minor_steps = 0.005;

  
  // ODrive object
//  ODriveArduino odrive(odrive_serial);
  odrive_serial.begin(115200);
}

void Motor_odrive_differential::update_motor_vel(){
  vel_A = final_multipler*(-1.0*vel_R3 + vel_T3);
  vel_B = final_multipler*(-1.0*vel_R3 - vel_T3);
}

void Motor_odrive_differential::publish_motor_pos(){
  pos_R3_msg.data = (-0.5*pos_A - 0.5*pos_B)/final_multipler;
  pub_R3.publish(&pos_R3_msg);

  pos_T3_msg.data = (0.5*pos_A - 0.5*pos_B)/final_multipler;
  pub_T3.publish(&pos_T3_msg);
}

void Motor_odrive_differential::step(){
  if(vel_A != 0){
    unsigned long current_time_A = millis();

    if (((current_time_A - previous_time_A)/1000.0) > (minor_steps/float(abs(vel_A)))){
      pos_A = pos_A + sgn(vel_A)*minor_steps;
      odrive.SetPosition(0, pos_A);
      
      publish_motor_pos();
      previous_time_A = millis();
    }
  }

  if(vel_B != 0){
    unsigned long current_time_B = millis();
    if (((current_time_B - previous_time_B)/1000.0) > (minor_steps/float(abs(vel_B)))){
      pos_B = pos_B + sgn(vel_B)*minor_steps;
      odrive.SetPosition(0, pos_B);
      
      publish_motor_pos();
      previous_time_B = millis();
    }
  }
}

void Motor_odrive_differential::message_R3_callback( const std_msgs::Float32& vel) {
  vel_R3 = vel.data;
  update_motor_vel();
}

void Motor_odrive_differential::message_T3_callback( const std_msgs::Float32& vel) {
  vel_T3 = vel.data;
  update_motor_vel();
}
