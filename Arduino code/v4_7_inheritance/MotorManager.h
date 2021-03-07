#ifndef MotorManager_h
#define MotorManager_h

/*provides manager MotorManager control thru single steps*/

#include <ros.h>
#include <v4_6dof/VelGap.h>
#include "MotorStepper.h"
#include "MotorDifferential.h"
#include "MotorAxis.h"

class MotorManager {
  public:
    MotorManager(MotorStepper* R1, MotorStepper* T1, MotorStepper* T2, MotorStepper* R2, MotorDifferential* T3R3);

    void messageCallback( const v4_6dof::VelGap& vel_six);

    void pushToQueue(float incoming_vels[6], uint32_t incoming_gap );
    void pubPoss();
    void checkSteps();

    ros::Subscriber<v4_6dof::VelGap, MotorManager> sub;
    ros::Publisher pub;

  private:
    MotorStepper *motorlist[4];
    MotorDifferential *motordiff;
    v4_6dof::VelGap pos_msg;


};

#endif
