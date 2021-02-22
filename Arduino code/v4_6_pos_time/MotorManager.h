#ifndef MotorManager_h
#define MotorManager_h

/*provides manager MotorManager control thru single steps*/

#include <ros.h>
#include <v4_6dof/PosTime.h>
#include "MotorStepper.h"
#include "MotorDifferential.h"

class MotorManager {
  public:
    MotorManager(MotorStepper* R1, MotorStepper* T1, MotorStepper* T2, MotorStepper* R2, MotorDifferential* T3R3);

    void messageCallback( const v4_6dof::PosTime& vel_six);

    void setVels(float incoming_vels[]);
    void pubPoss();
    void checkSteps();

    ros::Subscriber<v4_6dof::PosTime, MotorManager> sub;
    ros::Publisher pub;

  private:
    MotorStepper *motorlist[4];
    MotorDifferential *motordiff;
    v4_6dof::PosTime pos_msg;


};

#endif
