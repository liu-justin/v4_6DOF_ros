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
    MotorManager(MotorStepper& R1): motor_R1(R1) {}
//    MotorManager(MotorStepper* R1);

    MotorStepper motor_list[4];
    MotorStepper motor_R1;
    
  private:
    


};

#endif
