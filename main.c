#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include "include/Class5LinuxApp.h"


int main(void) 
{ 
	int Port =0;
        int SMX = 0; 
	int SMXE = 0;
	int Firmware = 0;
    	int x;
	int Baud = 115200;
	// int value;
	char temp[16]; 	// needed for some tests
	long value=0; 	// needed for some tests
    	puts("Start.."); 
    	Port = OpenPort();
   	SMX = open("testSMX.smx", O_RDWR | O_NOCTTY);
  	SMXE = open("testSMXe.smxe", O_RDWR | O_NOCTTY);
	Firmware = open("class_5_0_4_31.hez", O_RDWR | O_NOCTTY);
    	if (Port < 0|| SMX < 0 || SMXE < 0 || Firmware <0)
    	{
		printf("%d\n",Port);
		printf("%d\n",SMX);
		printf("%d\n",SMXE);
		printf("%d\n",Firmware);
        	puts("error opening a file");
		
	}
//	AddressRS232(Port, Baud);
// Tests for different functions in Class5LinuxApp.c
// echo = 1, ECHO_ON
//-----------------------------------------------------
//	x = DownloadSmxFile(Port, SMX, Baud, 1);
//	x = DownloadSmxeFile(Port,SMXE,Baud,1);
//      x = UploadSMX(Port,Baud, 1);
//	x = UploadSMS(Port,Baud,1);
//	x = DownloadFirmware(Port,Firmware, 38400,1);
//	x = WriteCommand(Port, Baud, "RPA ", 4);
	x = AddressRS232(Port, Baud);
	x = WriteCommand(Port, Baud, "MV ", 1, 0x85);
	x = WriteCommand(Port, Baud, "VT=10000 ", 1, 0x85);
	x = WriteCommand(Port, Baud, "G ", 1, 0x85);
	// x = WriteCommand(Port, Baud, "X ", 0, 0x85);
//	x = GetFirmwareVersion(Port, Baud, temp ,1);
//	x = GetSerial(Port, Baud, temp,1);		
//	x = GetProdDate(Port, Baud, temp, 1);
//	x = EstLinkNew(Port, Baud);
//	x = DetectRS232(Port, Baud);
	//x = GetProdDate(Port, Baud, temp, 1);
	value = GetValue(Port, Baud, "RVT ", value, 1);
	printf("%s%ld\n", "value: ", value);
	value = GetValue(Port, Baud, "RVT ", value, 1);
	printf("%s%ld\n", "value: ", value);
	value = GetValue(Port, Baud, "RVT ", value, 1);
	printf("%s%ld\n", "value: ", value);

//	x = DetectRS485(Port, Baud);
	if ( x >= 0)
   	{
		puts("Complete");
   	}
   	else
   	{
		printf("Error: %d\n", x);
	   
   	}
	ClosePort(Port);
	ClosePort(SMX);
	ClosePort(SMXE);
   	ClosePort(Firmware);   
	
   	return 0; 
}
