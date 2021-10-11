#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "smartmotors_linux/command.h"
#include <stdio.h>
#include <wrapper.h>

SmartMotors s;

void callback(const smartmotors_linux::command::ConstPtr& msg){

  char* line;
  int num;
  line = (char*)msg->motorcommand.c_str(); 
  num = msg->motorno;
  // std::cout << msg->data.c_str() << std::endl;
  std::cout << num << line << std::endl;
  s.command(line, num);

}

int main(int argc, char **argv){


  ros::init(argc, argv, "smartmotors");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("smartmotor_command", 1, callback);

  ros::spin();

}
