#include "MotorDifferential.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <HardwareSerial.h>
#include <ODriveArduino.h>

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

MotorDifferential::MotorDifferential(HardwareSerial& odrive_serial)
  : odrive(odrive_serial)
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
  rad_to_rev = 1 / (2 * PI);
  final_multipler = speed_ratio * rad_to_rev;
  minor_steps = 0.005;

  odrive_serial.begin(115200);
}

void MotorDifferential::setVelT3(float new_vel) {
  vel_T3 = new_vel;
  updateVelAB();
}

void MotorDifferential::setVelR3(float new_vel) {
  vel_R3 = new_vel;
  updateVelAB();
}

void MotorDifferential::updateVelAB() {
  vel_A = final_multipler * (-1.0 * vel_R3 + vel_T3);
  vel_B = final_multipler * (-1.0 * vel_R3 - vel_T3);
}

float MotorDifferential::getPosT3() {
  return (-0.5*pos_A - 0.5*pos_B)/final_multipler;
}

float MotorDifferential::getPosR3() {
  return (0.5*pos_A - 0.5*pos_B)/final_multipler;
}

//void MotorDifferential::publish_motor_pos(){
//  pos_R3_msg.data = (-0.5*pos_A - 0.5*pos_B)/final_multipler;
//  pub_R3.publish(&pos_R3_msg);
//
//  pos_T3_msg.data = (0.5*pos_A - 0.5*pos_B)/final_multipler;
//  pub_T3.publish(&pos_T3_msg);
//}

void MotorDifferential::checkStep(unsigned long current_time) {
  if (vel_A != 0) {
    if (((current_time - previous_time_A) / 1000.0) > (minor_steps / float(abs(vel_A)))) {
      pos_A = pos_A + sgn(vel_A) * minor_steps;
      odrive.SetPosition(0, pos_A);

//      previous_time_A = current_time;
      previous_time_A += (minor_steps / float(abs(vel_A)));
    }
  }

  if (vel_B != 0) {
    if (((current_time - previous_time_B) / 1000.0) > (minor_steps / float(abs(vel_B)))) {
      pos_B = pos_B + sgn(vel_B) * minor_steps;
      odrive.SetPosition(1, pos_B);

//      previous_time_B = current_time;
      previous_time_B = (minor_steps / float(abs(vel_B)));
    }
  }
}
