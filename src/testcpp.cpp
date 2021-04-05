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
  // int value;
	char temp[16]; 	// needed for some tests
	long value=0; 	// needed for some tests
  puts("Start..");
  Port = OpenPort();
  // SMX = open("testSMX.smx", O_RDWR | O_NOCTTY);
  // SMXE = open("testSMXe.smxe", O_RDWR | O_NOCTTY);
  // Firmware = open("class_5_0_4_31.hez", O_RDWR | O_NOCTTY);
    // if (Port < 0|| SMX < 0 || SMXE < 0 || Firmware <0)
    // {
    //   cout << Port << endl;
    // }

  int a = 0x80;
  int aa = 0x81;
  cout << a << endl;
  char* b = "MP ";
  char* c = "PT=1000 ";
  char* d = "G ";

  cout << Port << endl;
  // DetectRS232(Port, Baud);
  // cout << "Detection Complete" << endl;
  // AddressRS232(Port, Baud);
  // cout << "Addresing Complete" << endl;
  WriteCommand(Port, Baud, b , 1, a);
  WriteCommand(Port, Baud, "PT=5000 ", 1, aa);
  WriteCommand(Port, Baud, "G ", 1 , aa);




  return 0;

}
