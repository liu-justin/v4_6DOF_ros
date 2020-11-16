#include <Motor.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;

Motor R1(2,3,-45, 45, "chatter_motorR1");
Motor T1(4,5,-180, 180, "chatter_motorT1");
Motor T2(6,7,-180, 180, "chatter_motorT2");
Motor R2(8,9, -180, 180, "chatter_motorR2");

//ros::Subscriber<std_msgs::Int8> sub1("chatter_motorR1", messageCb_R1 );
//ros::Subscriber<std_msgs::Int8> sub2("chatter_motorT1", messageCb_T1 );
//ros::Subscriber<std_msgs::Int8> sub3("chatter_motorT2", messageCb_T2 );
//ros::Subscriber<std_msgs::Int8> sub4("chatter_motorR2", messageCb_R2 );

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
//  nh.subscribe(R1.sub);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  R1.step(1);
  T1.step(1);
  T2.step(1);
  R2.step(1);
  nh.spinOnce();
}
