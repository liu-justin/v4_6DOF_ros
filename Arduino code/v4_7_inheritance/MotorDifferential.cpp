#include "MotorDifferential.h"

#include <ros.h>

#include <HardwareSerial.h>
#include <ODriveArduino.h>

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

MotorDifferential::MotorDifferential(HardwareSerial& odrive_serial, int tilt_lower, int tilt_upper, int rot_lower, int rot_upper, float step_size, float multipler)
  : odrive(odrive_serial)
  , tilt(tilt_lower, tilt_upper)
  , rotation(rot_lower, rot_upper)
{
  vel_A = 0.0;
  vel_B = 0.0;
  pos_A = 0.0;
  pos_B = 0.0;

  speed_ratio = multipler;
  rad_per_step = step_size;

  odrive_serial.begin(115200);
}

//float MotorDifferential::getPosTilt() {
//  return (-0.5*pos_A - 0.5*pos_B)/final_multipler;
//}
//
//float MotorDifferential::getPosRot() {
//  return (0.5*pos_A - 0.5*pos_B)/final_multipler;
//}

void MotorDifferential::checkStep(unsigned long current_time) {
  // previously I changed this vel whenever a tilt, rot vel changed, now I have to sample on every loop
  vel_A = speed_ratio * (-1.0 * rotation.getVel() + tilt.getVel());
  if (vel_A != 0) {
    if ((float(current_time - previous_time_A) / 1000000.0) > (rad_per_step / float(abs(vel_A)))) {
      pos_A = pos_A + sgn(vel_A) * rad_per_step;
      // odrive position is in terms of revolution
      odrive.SetPosition(0, pos_A/(2*PI));

      previous_time_A = current_time;
    }
  }

  vel_B = speed_ratio * (-1.0 * rotation.getVel() - tilt.getVel());
  if (vel_B != 0) {
    if ((float(current_time - previous_time_B) / 1000000.0) > (rad_per_step / float(abs(vel_B)))) {
      pos_B = pos_B + sgn(vel_B) * rad_per_step;
      odrive.SetPosition(1, pos_B/(2*PI));

      previous_time_B = current_time;
    }
  }
}

//void MotorDifferential::publish_motor_pos(){
//  pos_R3_msg.data = (-0.5*pos_A - 0.5*pos_B)/final_multipler;
//  pub_R3.publish(&pos_R3_msg);
//
//  pos_T3_msg.data = (0.5*pos_A - 0.5*pos_B)/final_multipler;
//  pub_T3.publish(&pos_T3_msg);
//}
