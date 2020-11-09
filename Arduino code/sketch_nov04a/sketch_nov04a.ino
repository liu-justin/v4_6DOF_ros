#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

void messageCb_R1( const std_msgs::Int8& toggle_msg){
  digitalWrite(2, HIGH-digitalRead(13));   // blink the led
}
void messageCb_T1( const std_msgs::Int8& toggle_msg){
  digitalWrite(4, HIGH-digitalRead(13));   // blink the led
}
void messageCb_T2( const std_msgs::Int8& toggle_msg){
  digitalWrite(6, HIGH-digitalRead(13));   // blink the led
}
void messageCb_R2( const std_msgs::Int8& toggle_msg){
  digitalWrite(8, HIGH-digitalRead(13));   // blink the led
}
void messageCb_T3( const std_msgs::Int8& toggle_msg){
  digitalWrite(10, HIGH-digitalRead(13));   // blink the led
}
void messageCb_R3( const std_msgs::Int8& toggle_msg){
  digitalWrite(12, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Int8> sub("chatter_motorR1", messageCb_R1 );
ros::Subscriber<std_msgs::Int8> sub("chatter_motorT1", messageCb_T1 );
ros::Subscriber<std_msgs::Int8> sub("chatter_motorT2", messageCb_T2 );
ros::Subscriber<std_msgs::Int8> sub("chatter_motorR2", messageCb_R2 );
ros::Subscriber<std_msgs::Int8> sub("chatter_motorT3", messageCb_T3 );
ros::Subscriber<std_msgs::Int8> sub("chatter_motorR3", messageCb_R3 );


void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);

  nh.initNode();
  nh.subscribe("chatter_motorR1");
  nh.subscribe("chatter_motorT1");
  nh.subscribe("chatter_motorT2");
  nh.subscribe("chatter_motorR2");
  nh.subscribe("chatter_motorT3");
  nh.subscribe("chatter_motorR3");
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  
}
