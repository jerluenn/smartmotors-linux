#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <Class5LinuxApp.h>
#include <iostream>
#include <wrapper.h>
#include <chrono>
#include <cstdlib>

using namespace std;

SmartMotors::SmartMotors()
{

  Port = OpenPort();
  int SMX = 0; 
	int SMXE = 0;
	int Firmware = 0;
  int flag = 0;
  SMX = open("/home/jerluennn/catkin_ws/src/smartmotors-linux/testSMX.smx", O_RDWR | O_NOCTTY);
  SMXE = open("/home/jerluennn/catkin_ws/src/smartmotors-linux/testSMXe.smxe", O_RDWR | O_NOCTTY);
	Firmware = open("/home/jerluennn/catkin_ws/src/smartmotors-linux/class_5_0_4_31.hez", O_RDWR | O_NOCTTY);
    	if (Port < 0|| SMX < 0 || SMXE < 0 || Firmware <0)
    	{
          printf("%d\n",Port);
          printf("%d\n",SMX);
          printf("%d\n",SMXE);
          printf("%d\n",Firmware);
        	puts("error opening a file");
    	}
  AddressRS232(Port, Baud);
  // std::atexit(exiting);
  // Baud = 115200; 
  // flag = SetBaudrate(Port, Baud);
  // if (flag == -1) {

  //         puts("Set baud rate failed.");

  // }

  // else if (flag == 0) {


  //         printf("Baud rate set to %d\n", Baud);

  // }


}

// void Smartmotors::exiting() 

// {

//   ClosePort(Port);

// }

  

void SmartMotors::read(char* line, int MotorNo) 

{

  long value = 0;
  long v = 0;
  int motor;
  motor = Base + MotorNo;
  value = GetValue(Port, Baud, line, v, 1);
  std::cout << value; 

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
