#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <Class5LinuxApp.h>
#include <iostream>
#include <wrapper.h>

using namespace std;

SmartMotors::SmartMotors(){

  Port = OpenPort();

}

void SmartMotors::command(char* line, int MotorNo){

  int number = Base + MotorNo;
  WriteCommand(Port, Baud, line, 1, number);

}

void SmartMotors::commandAll(char* line){

  WriteCommand(Port, Baud, line, 1, Base);

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
