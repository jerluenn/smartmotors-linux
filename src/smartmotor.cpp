#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "smartmotors_linux/command.h"
#include "smartmotors_linux/arraycommand.h"
#include <stdio.h>
#include <wrapper.h>
#include <string.h>
#include <stack>
#include <chrono>

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

void array_callback(const smartmotors_linux::arraycommand::ConstPtr& msg) {

  std::string str;
  std::string gostr("G ");
  int arraySize = msg->motorarrayvalues.size();
  char* line;
  int num;
  auto t_start = std::chrono::high_resolution_clock::now();


  for(int i = 0; i < arraySize; i++ ){

    num = i+1;
    str = std::to_string(msg->motorarrayvalues[i]);
    line = (char*)(msg->motorcommand + '=' + str + ' ').c_str();
    std::cout << num << line << std::endl;
    std::cout << num << (char*)gostr.c_str() << std::endl;
    s.command(line, num);
    s.command((char*)gostr.c_str(), num);

  }

  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
  std::cout << "Time elapsed: " << elapsed_time_ms/1000 << std::endl;

}

int main(int argc, char **argv){

  auto t_start = std::chrono::high_resolution_clock::now();

  ros::init(argc, argv, "smartmotors");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("smartmotor_command", 1, callback, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber sub2 = n.subscribe("smartmotor_array_command", 1, array_callback, ros::TransportHints().tcpNoDelay(true));

  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();

  // ros::spin();

}
