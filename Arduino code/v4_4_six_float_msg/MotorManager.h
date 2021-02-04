#ifndef MotorManager_h
#define MotorManager_h

/*provides manager MotorManager control thru single steps*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include "MotorStepper.h"

class MotorManager {
  public:
    //    MotorManager(MotorStepper& R1, MotorStepper& T1, MotorStepper& T2, MotorStepper& R2):
    //    motor_R1(R1),
    //    motor_T1(T1),
    //    motor_T2(T2),
    //    motor_R2(R2){
    //
    //    };
    MotorManager(MotorStepper* R1, MotorStepper* T1, MotorStepper* T2, MotorStepper* R2) {
      motorlist[0] = R1;
      motorlist[1] = T1;
      motorlist[2] = T2;
      motorlist[3] = R2;
    };


  private:
    //    MotorStepper motor_R1;
    //    MotorStepper motor_T1;
    //    MotorStepper motor_T2;
    //    MotorStepper motor_R2;
    MotorStepper *motorlist[4];

    ros::Subscriber sub;
    ros::Publisher pub;


};

#endif
