
#include "MotorManager.h"

#include <ros.h>

#include "MotorStepper.h"


MotorManager::MotorManager(MotorStepper* R1, MotorStepper* T1, MotorStepper* T2, MotorStepper* R2)
  : sub("vel_six_chatter", &MotorManager::messageCallback, this)
  , pub("pos_six_chatter", &pos_msg)
{
  motorlist[0] = R1;
  motorlist[1] = T1;
  motorlist[2] = T2;
  motorlist[3] = R2;
}

void MotorManager::messageCallback( const v4_6dof::Float32List& vel_six) {
  setVels(vel_six.data);
}

void MotorManager::setVels(const float incoming_vels[6]) {
  for (int i = 0 ; i < 4; i++) {
    motorlist[i]->setVel(incoming_vels[i]);
  }
}

void MotorManager::pubPoss() {
  for (int i = 0; i < 4; i++) {
    pos_msg.data[i] = motorlist[i]->getPos();
  }

  pub.publish(&pos_msg);

}

void MotorManager::checkSteps() {
  unsigned long current_time = millis();

  for (int i = 0 ; i < 4; i++) {
    motorlist[i]->checkStep(current_time);
  }
}
