/*
 * Node used to test if the Arduino sketch that subscribes to the topic "/led_strip/turn_signal" is receiving messages. 
 * If connection is active and working, the LED lights should create a left turn signal. 
 */
 
#include "ros/ros.h"
#include "std_msgs/Char.h"

ros::Publisher signal_pub; 

int main(int argc, char **argv) {
  ros::init(argc, argv, "connection_test");
  ros::NodeHandle n; 
  
  signal_pub = n.advertise<std_msgs::Char>("/led_strip/turn_signal", 100); 
  
  std_msgs::Char msg;
  msg.data = '1';  
  while(ros::ok()) {
	ros::ok(); 
	signal_pub.publish(msg);   
  } 
  return 0;  
}
