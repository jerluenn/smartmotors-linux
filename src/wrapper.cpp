#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <Class5LinuxApp.h>
#include <iostream>
#include <wrapper.h>
#include <chrono>

using namespace std;

SmartMotors::SmartMotors()
{

  Port = OpenPort();
  AddressRS232(Port, Baud);

}

void SmartMotors::command(char *line, int MotorNo)
{

  int motor;
  motor = Base + MotorNo;
  WriteCommand(Port, Baud, line, 1, motor);

}

void SmartMotors::commandAll(char *line)
{
  WriteCommand(Port, Baud, line, 1, 0);
}

void SmartMotors::tic() {
    t_start = std::chrono::high_resolution_clock::now();
}

void SmartMotors::toc() {

    t_end = std::chrono::high_resolution_clock::now();
    elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
    std::cout << "Time elapsed: "
              << elapsed_time_ms
              << std::endl;

}



// class SmartMotors {
//
//   public:
//
//     int Port = 1;
//     int Baud = 9600;
//     int Base = 0x80;
//
//     SmartMotors(){
//
//       Port = OpenPort();
//
//     }
//
//     void command(char* line, int MotorNo){
//
//       int number = Base + MotorNo;
//       WriteCommand(Port, Baud, line, 1, number);
//
//
//     }
//
//     void commandAll(char* line) {
//
//       WriteCommand(Port, Baud, line, 1, Base);
//
//     }
//
// };
