#ifndef Motor_manager_h
#define Motor_manager_h

/*provides manager Motor_manager control thru single steps*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include "Motor_stepper.h"

class Motor_manager {
  public:
//    Motor_manager(&Motor_stepper R1, &Motor_stepper T1, &Motor_stepper T2, &Motor_stepper R2);
    Motor_manager();
    int getPos();
    int getVel();
    void step();

    unsigned long previousTime;

    &Motor_stepper motor_list[4] = {};
    
  private:
    


};

#endif
