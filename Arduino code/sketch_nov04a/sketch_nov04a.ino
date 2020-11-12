#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

void messageCb_R1( const std_msgs::Int8&){
  digitalWrite(2, HIGH-digitalRead(13));   // blink the led
}
//void messageCb_T1( const std_msgs::Int8&){
//  digitalWrite(4, HIGH-digitalRead(13));   // blink the led
//}
//void messageCb_T2( const std_msgs::Int8&){
//  digitalWrite(6, HIGH-digitalRead(13));   // blink the led
//}
//void messageCb_R2( const std_msgs::Int8&){
//  digitalWrite(8, HIGH-digitalRead(13));   // blink the led
//}
//void messageCb_T3( const std_msgs::Int8&){
//  digitalWrite(10, HIGH-digitalRead(13));   // blink the led
//}
//void messageCb_R3( const std_msgs::Int8&){
//  digitalWrite(12, HIGH-digitalRead(13));   // blink the led
//}

ros::Subscriber<std_msgs::Int8> sub_R1("chatter_motorR1", messageCb_R1 );
//ros::Subscriber<std_msgs::Int8> sub_T1("chatter_motorT1", messageCb_T1 );
//ros::Subscriber<std_msgs::Int8> sub_T2("chatter_motorT2", messageCb_T2 );
//ros::Subscriber<std_msgs::Int8> sub_R2("chatter_motorR2", messageCb_R2 );
//ros::Subscriber<std_msgs::Int8> sub_T3("chatter_motorT3", messageCb_T3 );
//ros::Subscriber<std_msgs::Int8> sub_R3("chatter_motorR3", messageCb_R3 );


void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);

  nh.initNode();
  nh.subscribe("sub_R1");
  //nh.subscribe("sub_T1");
  //nh.subscribe("sub_T2");
  //nh.subscribe("sub_R2");
  //nh.subscribe("sub_T3");
  //nh.subscribe("sub_R3");
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  
}
