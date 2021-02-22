
#include "MotorManager.h"

#include <ros.h>

#include "MotorStepper.h"


MotorManager::MotorManager(MotorStepper* R1, MotorStepper* T1, MotorStepper* T2, MotorStepper* R2, MotorDifferential* T3R3)
  : sub("vel_six_chatter", &MotorManager::messageCallback, this)
  , pub("pos_six_chatter", &pos_msg)
{
  motorlist[0] = R1;
  motorlist[1] = T1;
  motorlist[2] = T2;
  motorlist[3] = R2;
  motordiff = T3R3;
}

void MotorManager::messageCallback( const v4_6dof::Float32List& vel_six) {
  setVels(vel_six.data);
  // change this, need to establish goal pos, and the time to get there
}

void MotorManager::setVels(float incoming_vels[6]) {
  for (int i = 0 ; i < 4; i++) {
    motorlist[i]->setVel(incoming_vels[i]);
  }
  motordiff->setVelT3(incoming_vels[4]);
  motordiff->setVelR3(incoming_vels[5]);
}

void MotorManager::pubPoss() {
  for (int i = 0; i < 4; i++) {
    pos_msg.data[i] = motorlist[i]->getPos();
  }
  pos_msg.data[4] = motordiff->getPosT3();
  pos_msg.data[5] = motordiff->getPosR3();
  pub.publish(&pos_msg);

}

void MotorManager::checkSteps() {
  unsigned long current_time = micros();

  for (int i = 0 ; i < 4; i++) {
    motorlist[i]->checkStep(current_time);
  }
  motordiff->checkStep(current_time);
  pubPoss();
}
