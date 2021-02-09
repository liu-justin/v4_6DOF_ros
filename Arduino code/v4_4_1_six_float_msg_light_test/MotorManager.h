#ifndef MotorManager_h
#define MotorManager_h

/*provides manager MotorManager control thru single steps*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <v4_6dof/Float32List.h>

#include "MotorStepper.h"

class MotorManager {
  public:

    MotorManager(MotorStepper* R1, MotorStepper* T1, MotorStepper* T2, MotorStepper* R2);

    void messageCallback( const v4_6dof::Float32List& vel_six);

    void setVels(float incoming_vels[]);
    void checkSteps();

    ros::Subscriber<v4_6dof::Float32List, MotorManager> sub;
    ros::Publisher pub;
    void pubPoss();

  private:
    MotorStepper *motorlist[4];
    v4_6dof::Float32List pos_msg;


};

#endif
