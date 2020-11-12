/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;


void messageCb_R1( const std_msgs::Int8&){
  digitalWrite(2, HIGH);   // blink the led
}

void messageCb_T1( const std_msgs::Int8&){
  digitalWrite(4, HIGH);   // blink the led
}

ros::Subscriber<std_msgs::Int8> sub1("chatter_motorR1", messageCb_R1 );
ros::Subscriber<std_msgs::Int8> sub2("chatter_motorT1", messageCb_T1 );

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
}

void loop()
{
  nh.spinOnce();
}
