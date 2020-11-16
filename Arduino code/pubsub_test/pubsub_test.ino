/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;


void messageCb_R1( const std_msgs::Int8& vel){
  if (vel.data == 0) {
    digitalWrite(2, LOW);
  }
  else {
    digitalWrite(2, HIGH); 
  }
}

void messageCb_T1( const std_msgs::Int8& vel){
  if (vel.data == 0) {
    digitalWrite(4, LOW);
  }
  else {
    digitalWrite(4, HIGH); 
  }
}

void messageCb_T2( const std_msgs::Int8& vel){
  if (vel.data == 0) {
    digitalWrite(6, LOW);
  }
  else {
    digitalWrite(6, HIGH); 
  }
}

void messageCb_R2( const std_msgs::Int8& vel){
  if (vel.data == 0) {
    digitalWrite(8, LOW);
  }
  else {
    digitalWrite(8, HIGH); 
  }
}

void messageCb_T3( const std_msgs::Int8& vel){
  if (vel.data == 0) {
    digitalWrite(10, LOW);
  }
  else {
    digitalWrite(10, HIGH); 
  }
}

void messageCb_R3( const std_msgs::Int8& vel){
  if (vel.data == 0) {
    digitalWrite(12, LOW);
  }
  else {
    digitalWrite(12, HIGH); 
  }
}

ros::Subscriber<std_msgs::Int8> sub1("chatter_motorR1", messageCb_R1 );
ros::Subscriber<std_msgs::Int8> sub2("chatter_motorT1", messageCb_T1 );
ros::Subscriber<std_msgs::Int8> sub3("chatter_motorT2", messageCb_T2 );
ros::Subscriber<std_msgs::Int8> sub4("chatter_motorR2", messageCb_R2 );
ros::Subscriber<std_msgs::Int8> sub5("chatter_motorT3", messageCb_T3 );
ros::Subscriber<std_msgs::Int8> sub6("chatter_motorR3", messageCb_R3 );

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
}

void loop()
{
  nh.spinOnce();
}
