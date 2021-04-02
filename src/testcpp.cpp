#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <Class5LinuxApp.h>
#include <iostream>

using namespace std;

int main() {

  int Port =0;
  int SMX = 0;
  int SMXE = 0;
  int Firmware = 0;
  int x;
  int Baud = 9600;
  int value;
  cout << "test" << endl;
//	char temp[16]; 	// needed for some tests
//	long value=0; 	// needed for some tests
  // puts("Start..");
  Port = OpenPort();
  SMX = open("testSMX.smx", O_RDWR | O_NOCTTY);
  SMXE = open("testSMXe.smxe", O_RDWR | O_NOCTTY);
  Firmware = open("class_5_0_4_31.hez", O_RDWR | O_NOCTTY);
    if (Port < 0|| SMX < 0 || SMXE < 0 || Firmware <0)
    {
      cout << "error opening file." << endl;
    }

  return 0;

}
