#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include <stdio.h>
#include <wrapper.h>

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg){

  std::cout << msg->data[1] << std::endl;
  std::cout << line << std::endl;
  char* line2 = "G ";
  int a = 1;
  SmartMotors s;
  s.command(line, a);
  s.command(line2, a);

}

int main(int argc, char **argv){


  ros::init(argc, argv, "smartmotors");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("smartmotor_command", 1, callback);

  ros::spin();

}
