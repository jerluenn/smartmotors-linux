#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>

int OpenPort(void);
void ClosePort(int Port);
int SetBaudrate(int Port, int Baud);
int DownloadSmxFile(int Port, int SMX, int Baud, char echo);
int DownloadSmxeFile(int Port, int SMXE, int Baud, char echo);
int DownloadFirmware(int Port, int Firmware, int Baud, char ehco);
int UploadSMS(int Port, int Baud,  char echo);
int UploadSMX(int Port, int Baud, char echo);
int WriteCommand(int Port, int Baud, char* Command,char echo, int MotorNo);
// int WriteCommand(int Port, int Baud, char* Command,char echo);
int GetFileName(int Port, char* Filename, char echo);
int GetFirmwareVersion(int Port, int Baud, char* temp, char echo);
int GetSerial(int Port, int Baud, char* temp, char echo);
int GetProdDate(int Port, int Baud, char* temp, char echo);
int GetValue(int Port, int Baud, char* Command, long value, char echo, char* MotorNo);
int testfunc(int Port);
int testBaud(int Port, int Baud);
int EstLinkNew(int Port, int Baud);
int DetectRS232(int Port, int Baud);
int AddressRS232(int Port, int Baud);
int DetectRS485(int Port, int Baud);
int DriveReady(int Port, int Baud);

int testfunc(int Port)
{
	if(SetBaudrate(Port, 9600) <0)
	{
		puts("fail 1");
	}
	sleep(1);
	if(write(Port,"Ra ", 3) <0)
	{
		puts("fail 2");
	}
	sleep(1);
//	ClosePort(Port);
//	Port = OpenPort();
	if(SetBaudrate(Port, 38400) <0)
	{
		puts("fail 3");
	}
	sleep(1);
	if(write(Port,"Ra ", 3) <0)
	{
		puts("fail 4");
	}
	sleep(1);
	if(SetBaudrate(Port, 9600) <0)
	{
		puts("fail 5");
	}
	sleep(1);
	if(write(Port,"Ra ", 3) <0)
	{
		puts("fail 6");
	}
	return 0;
}
	// -------------------------------------------------------
	// // 'Openport()' - Open serial port 1.
	// //
	// // Returns the file descriptor on success or -1 on error.
	// // -------------------------------------------------------
	int OpenPort(void)
	{
		struct termios options;
		int Port = 0;

		Port =open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

		if (Port == -1)
		{
		
		puts("error opening Port ttyUSB0, now trying ttyUSB1");
		
			Port = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
			
			if (Port == -1)
			
			{
			
				puts("error opening Port ttyUSB1");
				return -1;
				
			}
			
			else 
			{
		
				puts("Port ttyUSB1 opened."); 
		
			}
		}
		
		
			
		

		fcntl(Port, F_SETFL,0);

		if(tcgetattr(Port, &options) <0)
		{
			return -1;
		}

		cfmakeraw(&options);

		options.c_cflag |= (CLOCAL | CREAD);    // Enable the reciever and set local mode
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CRTSCTS;		// Disable hardware flow control

		if(tcsetattr(Port,TCSANOW, &options) <0)
		{
			return -1;
		}

		return (Port);

	}

	// -------------------------------------------------------
	// // 'SetBaudrate()' - Set the baudrate to the serial port
	// //
	// // Returns  -1 on error.
	// // -------------------------------------------------------
	//
	int SetBaudrate(int Port, int Baud)
	{
		struct termios options;

		if(tcgetattr(Port, &options) <0)
		{
			return -1;
		}
		switch(Baud)
		{
			case 2400:
				if(cfsetispeed(&options,B2400)== -1)
				{
					return -1;
				}
				if(cfsetospeed(&options,B2400) ==-1)
				{
					return -1;
				}
			case 4800:
				if(cfsetispeed(&options,B4800)== -1)
				{
					return -1;
				}
				if(cfsetospeed(&options,B4800) ==-1)
				{
					return -1;
				}
			case 9600:
				if(cfsetispeed(&options,B9600)== -1)
				{
					return -1;
				}
				if(cfsetospeed(&options,B9600)==-1)
				{
					return -1;
				}
				break;
			case 19200:
				if(cfsetispeed(&options,B19200)== -1)
				{
					return -1;
				}
				if(cfsetospeed(&options,B19200)==-1)
				{
					return -1;
				}
				break;
			case 38400:
				if(cfsetispeed(&options,B38400)== -1)
				{
					return -1;
				}

			if(cfsetospeed(&options,B38400)==-1)
			{
				return -1;
			}
			break;

		case 57600:
			if(cfsetispeed(&options,B57600)== -1)
			{
				return -1;
			}
			if(cfsetospeed(&options,B57600)==-1)
			{
				return -1;
			}
			break;

		case 115200:
			if(cfsetispeed(&options,B115200)== -1)
			{
				return -1;
			}
			if(cfsetospeed(&options,B115200)==-1)
			{
				return -1;
			}
			break;
		default:
			return -1;
			break;
	}
/////////////////////////////////////////////////////////////////////////////
//TCSANOW   - Change is immediate
//TCSADRAIN - Change occurs after ouput written to fd has been tx
//	    - Used when changing params affecting output
//TCSAFLUSH - Change occurs after ouput writtn to object referred by fd is tx
//	    - All inputs recieved but not read are discarded
/////////////////////////////////////////////////////////////////////////////
	if(tcsetattr(Port,TCSAFLUSH, &options) < 0)
	{
		return -1;
	}
	return 0;
}

void ClosePort(int Port)
{
	close(Port);
}


// -------------------------------------------------------
// // 'DownloadSmxFile()' - Download .smx file to motor
// // Passes in the Serial Port, .smx file and baudrate
// // Returns the file descriptor on success or -1 on error.
// // -----------------------------------------------------
int DownloadSmxFile(int Port, int SMX, int Baud, char echo)
{
	char buf,resp;
	long RemainingBytes =(long) lseek(SMX, 0, SEEK_END);
	char buffer[RemainingBytes];
	long i=0;

        struct termios options;

	if( SetBaudrate(Port, Baud) != 0)
    	{
		puts("error setting baudrate");
		return -99;
    	}


	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSANOW, &options) <0)
		{
			return -1;
		}
	}

	while(read(Port, (unsigned char*)&buf, 1) >0)
	{
	}

	if (write(Port,"LOAD ",5)<= 0)
	{

		return -1;
	}

	if(echo)
	{
		for (i =0; i<5; i++)
		{
			read(Port,(unsigned char*)&buf,1);
		}
	}
	if(read(Port,(unsigned char *)&resp ,1)< 0)
	{
		return -2;
	}
	if (resp!= 0x06)
	{
		return -3;
	}
	lseek(SMX,0, SEEK_SET);
	if(read(SMX,(unsigned char*)&buffer, RemainingBytes)<0)
	{
		return -1;
	}
	i = 0;
	while (RemainingBytes >0)
	{

		long bytes_sent =  32;
		if(RemainingBytes < 32)
		{
			bytes_sent = RemainingBytes;
		}
		RemainingBytes -= bytes_sent;
		if (write(Port,(unsigned char*)&buffer[i], bytes_sent) < 0)
		{


			return -5;
		}
		i += bytes_sent;
		while(read(Port, (unsigned char*)&buf, 1)>0)
		{
		}
		if(bytes_sent == 32)
		{

			if(read(Port,(unsigned char*)&resp, 1) <0)
			{
				return -6;
			}

			if(resp != 0x06)
			{
				return -7;
			}
		}


	}

	if (write(Port, "\xFF\xFF\x20",3)<=0)
	{

		return -8;
	}
	return 0;
}

// -------------------------------------------------------
// // 'DowloadSmxeFile()' - Download .smxe to motor
// // Passes  in the Serial Port, ,.smxe file and baudrate
// // Returns the file descriptor on success or -1 on error.
// // -----------------------------------------------------
int DownloadSmxeFile(int Port, int SMXE, int Baud, char echo)
{
	char buf,resp;
	long RemainingBytes =(long) lseek(SMXE, 0, SEEK_END);
	char buffer[RemainingBytes];
	long i=0;

	struct termios options;

	if( SetBaudrate(Port, Baud) != 0)
    	{
		puts("error setting baudrate");
		return -99;
    	}



	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSANOW, &options) <0)
		{
			return -1;
		}
	}
	while(read(Port,(unsigned char*)&buf, 1) >0)
	{
	}
	if (write(Port,"LOAD(7) ",8)<= 0)
	{

		return -1;
	}
	if(echo)
	{
		for (i =0; i<8; i++)
		{
			read(Port,(unsigned char*)&buf,1);
		}
	}

	if(read(Port,(unsigned char *)&resp ,1)< 0)
	{
		return -2;
	}
	if (resp!= 0x06)
	{
		return -3;
	}

	lseek(SMXE, 0, SEEK_SET);
	if(read(SMXE,(unsigned char*)&buffer, RemainingBytes) <0)
	{
		return -4;
	}

	while (RemainingBytes >0)
	{
		long bytes_sent =  32;
		if(RemainingBytes < 32)
		{
			bytes_sent = RemainingBytes;
		}
		RemainingBytes -= bytes_sent;
		if (write(Port,(unsigned char*)&buffer[i], bytes_sent) < 0)
		{
			return -5;
		}
		i += bytes_sent;
		while(read(Port, (unsigned char*)&buf,1) >0)
		{
		}
		if(bytes_sent == 32)
		{
			if(read(Port,(unsigned char*)&resp, 1) <0)
			{

				return -6;
			}
			if(resp != 0x06)
			{
				return -7;
			}
		}
	}
	return 0;
}

// -------------------------------------------------------
// // 'DownloadFirmware()' - Download firmware to motor
// // Passes in the Serial Port, firmware file and baudrate
// // *** baudrate must be 38400 ***
// // Returns the file descriptor on success or -1 on error.
// // -----------------------------------------------------
int DownloadFirmware(int Port, int Firmware, int Baud, char echo)
{

	char buf;
	long fileLen =(long) lseek(Firmware, 0, SEEK_END);
	long i=0;
	char Readbuf[389];
	struct termios options;
	lseek(Firmware, 0, SEEK_SET);

	if( SetBaudrate(Port, Baud) != 0)
    	{
		puts("error setting baudrate");
		return -99;
    	}
	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;


		if(tcsetattr(Port, TCSAFLUSH, &options) <0)
		{
			return -1;
		}
	}

	while(read(Port, (unsigned char*)&buf,1)>0)
	{
	}
	char x = 0x80;
	if (write(Port, (unsigned char *)&x, 1) <=0)
	{
		return -2;
	}
	if(write(Port,"RSP2 ",5)==0)
	{
		return -3;
	}
	char rev;
	if(echo)
	{
		for(i=0;i<6;i++)
		{
			read(Port, (unsigned char*)&buf, 1);
		}
	}
	if(read(Port, (unsigned char*)&rev,1) <0)
	{
		return -44;
	}
	// allowed on rev 8 and up only
	if (rev < 56)
	{
		if(rev != 50)
		{
			puts("Rev not supported");
			return -55;
		}
	}

	sleep(.1);
	if (write(Port,"ECHO_OFF ",9)<= 0)
	{

		return -1;
	}
	if (write(Port,"CLASS5 ",7)<= 0)
	{

		return -1;
	}
//	char b=0x80;
	if( SetBaudrate(Port, 38400) != 0)
    	{
		puts("error setting baudrate");
		return -99;
    	}
	for(i =0; i< 100;i++)
	{

		if (write(Port,"START ",6)<= 0)
		{

			return -2;
		}
		sleep(.02);
	}
	sleep(.2);
	//encryption

	while(read(Port,(unsigned char*)&buf,1)>0)
	{
	}
	char a=0x0A;
	if (write(Port, (unsigned char *)&a, 1) <=0)
	{
		return -2;
	}

	if(read(Port,(unsigned char*)&buf, 1) == 0)
	{
		return -33;
	}
	if(buf != 0x01)
	{
	 	return -45;
	}




	read(Firmware, &Readbuf, 7);
	fileLen -= 7;
	while(read(Port,(unsigned char*)&buf,1)>0)
	{
	}
	if(write(Port, &Readbuf[0], 7) ==0)
	{
		return -5;
	}
	sleep(.01);

	if (read(Port, &buf, 1) ==0)
	{
		return -6;
	}
	if (buf != 0x01)
	{
		return -7;
	}



	while (fileLen >0)
	{
		buf =0;
		long length = 101;
		if(fileLen <101)
		{
			length = fileLen;
		}
		fileLen -=length;
		read(Firmware,(unsigned char*)&Readbuf, length);
		if(write(Port,(unsigned char*)&Readbuf[0], length)==0)
		{
			return -8;
		}
		if(length==101 )
		{

			if(read(Port, &buf,1)==0)
			{
				return -9;
			}
			if(buf != 0x01)
			{
				return -10;
			}
		}
	}
	return 0;


}

// -------------------------------------------------------
// // 'UploadSMS()' - Upload .sms file from motor
// // Passes in the Serial Port and baudrate
// // Returns the file descriptor on success or -1 on error.
// // -----------------------------------------------------
int UploadSMS(int Port, int  Baud, char echo)
{
	char buf, data;
	int i = 0;
	int j = 0;
	char a = 0x06;
	char end = 0xFF;
	char Filename[24];

	int SMS = 0;
	struct termios options;

	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -99;
    	}


	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;

	if(tcsetattr(Port, TCSANOW, &options) <0)
		{
			return -1;
		}
	}

	while(read(Port, (unsigned char*)&buf, 1)>0)
	{
	}
	if(write(Port,"EPTR=32340 VLD(ab[0],24) ",25)==0)
	{
		return -101;
	}
	if(echo)
	{
	 	for(i =0; i <25;i++)
	 	{
	 		read(Port,(unsigned char*)&buf, 1);
		}
	}

	if(GetFileName(Port,(char*)&Filename, echo)!=0)
	{
		return -99;
	}

	j =0;
	for (i =0; i <sizeof(Filename); i++)
	{

		if ( (int) Filename[i] <= 32)
		{
			break;
		}
	}
	Filename[i] = '.';
	Filename[i+1] = 's';
	Filename[i+2] = 'm';
	Filename[i+3] = 's';

	char Filename2[28];

	for(j=0;j<(i+4);j++)
	{
		Filename2[j]=Filename[j];
	}
	Filename2[i+4] = 0;

	SMS = open(Filename2, O_RDWR |O_APPEND | O_CREAT, 00007);

	while(read(Port,(unsigned char*)&buf,1)>0)
	{
	}
	if(write(Port, "UPLOAD ",7)==0)
	{
		return -1;
	}

	if(echo)
	{
		char up[7];
		if(read(Port,&up,7)==0)
		{
			return -2;
		}
		if(up[0] != 'U' || up[3] != 'O' || up[6] != ' ')
		{
			return -3;
		}

	}
	int count =0;
	while(read(Port, (unsigned char*)&data, 1) >0)
	{
		if( data == end)
		{
			ClosePort(SMS);
			return 0;
		}

		if(write(SMS, (unsigned char*)&data,1) ==0)
		{
			return -4;
		}
		count ++;
		if(count ==8)
		{
			if(write(Port,(unsigned char*)&a, 1)== 0)
			{
				return -5;
			}
			if(echo)
			{
				if(read(Port, &buf,1)==0)
				{
					return -6;
				}
				if(buf != 0x06)
				{
					return -7;
				}
			}
			count = 0;
		}
	}

	ClosePort(SMS);
	return 0;
}

// -------------------------------------------------------
// // 'UploadSMX()' - Upload .smx file from motor
// // Passes in the Serial Port and baudrate
// // Returns the file descriptor on success or -1 on error.
// // -----------------------------------------------------
int UploadSMX(int Port, int  Baud, char echo)
{
	char buf, data;
	int i = 0;
	int j = 0;
	char a = 0x06;
	char end = 0xFF;
	char Filename[24];

	int SMX = 0;
	struct termios options;

	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -99;
    	}


	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;

	if(tcsetattr(Port, TCSANOW, &options) <0)
		{
			return -1;
		}
	}

	while(read(Port, (unsigned char*)&buf, 1)>0)
	{
	}
	if(write(Port,"EPTR=32340 VLD(ab[0],24) ",25)==0)
	{
		return -101;
	}
	if(echo)
	{
	 	for(i =0; i <25;i++)
	 	{
	 		read(Port,(unsigned char*)&buf, 1);
		}
	}

	if(GetFileName(Port,(char*)&Filename, echo)!=0)
	{
		return -99;
	}
	j =0;
	for (i =0; i <sizeof(Filename); i++)
	{

		if ( (int) Filename[i] <= 32)
		{
			break;
		}
	}
	Filename[i] = '.';
	Filename[i+1] = 's';
	Filename[i+2] = 'm';
	Filename[i+3] = 'x';

	char Filename2[28];

	for(j=0;j<(i+4);j++)
	{
		Filename2[j]=Filename[j];
	}
	Filename[i+4] = 0;
	SMX = open(Filename2, O_RDWR |O_APPEND | O_CREAT, 00007);
	while(read(Port, (unsigned char*)&buf, 1)>0)
	{
	}
	if(write(Port, "UP ",3)==0)
	{
		return -1;
	}

	if(echo)
	{
		char up[3];
		if(read(Port,&up,3)==0)
		{
			return -2;
		}
		if(up[0] != 'U' || up[1] != 'P' || up[2] != ' ')
		{
			return -3;
		}

	}
	for ( i=0; i <9;i ++)
	{
		read(Port,(unsigned char*)&data,1);

		write(SMX,(unsigned char*)&data,1);
		write(Port, (unsigned char*)&a, 1);

	}
	if(echo)
	{
		if(read(Port,&buf,1) ==0)
		{
			return -66;
		}
		if(buf != 0x06)
		{
		 return -77;
		}
	}


	int count =0;
	while(read(Port, (unsigned char*)&data, 1) >0)
	{
		if( data == end)
		{
			ClosePort(SMX);
			return 0;
		}

		if(write(SMX, (unsigned char*)&data,1) ==0)
		{
			return -4;
		}
		count ++;
		if(count ==8)
		{
			if(write(Port,(unsigned char*)&a, 1)== 0)
			{
				return -5;
			}
			if(echo)
			{
				if(read(Port, &buf,1)==0)
				{
					return -6;
				}
				if(buf != a)
				{
					return -7;
				}
			}
			count = 0;
		}
	}

	ClosePort(SMX);
	return 0;
}

// // -------------------------------------------------------
// // 'WriteCommand()' - Write Command to Motor
// // Passes in the Serial Port, command  and baudrate, and MotorNo
// // as Hex values.
// // Returns the file descriptor on success or -1 on error.
// // -----------------------------------------------------

int WriteCommand(int Port, int Baud, char* Command,char echo, int MotorNo)
{
	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -1;

    	}



	write(Port, (unsigned char*)&MotorNo, 1);
	if(write(Port, Command,strlen(Command))==0)
	{
		return -1;
	}

	return 0;
}

// -------------------------------------------------------
// // 'GetFileName()' - Gets file name from the Motor
// // Passes in the Serial Port and baudrate
// // Returns the file descriptor on success or -1 on error.
// // Returns the name of the file in Filename paramater
// // -----------------------------------------------------
int GetFileName(int Port, char* Filename, char echo)
{
	char buf, f;
	char f1[12];
	char message[8];
	char nums[38] = "0123456789";
	char nums2[28] ="1011121314151617181920212223";
	int num1 =0;
	int k =0;
	int i;
	int j;
	int countx =0;
	int leave =1;


	for(i=0; i<24;i++)
	{

		if( i <10)
		{
			sprintf(message,"%s%c%s","Rab[",nums[i],"] ");

			if(write(Port,( char*)&message, 7)==0)
			{
				return -111;
			}
			if(echo)
			{
			 	for(k=0;k< 7;k++)
			 	{
					read(Port, (unsigned char*)&buf, 1);
				}
			}

			k=0;
		}
		else
		{
			sprintf(message, "%s%c%c%s","Rab[", nums2[countx], nums2[countx+1],"] ");




			if(write(Port,(unsigned char*)&message, 8)==0)
			{
				return -222;
			}
			if(echo)
			{
				for(k=0; k < 8;k++)
			 	{
					read(Port, (unsigned char*)&buf, 1);
				}
			}

			countx = countx +2;
		}
		j=0;
		leave =0;
		while((read(Port,(unsigned char*)&f,1)>0)   )
		{
			if( f != 0)
			{
				f1[j] = f;

			}
			j++;

		}
		sscanf(f1,"%d", &num1);
		if( num1 == 46 || num1 == 32)
		{
			break;
		}
		Filename[i] = (char) num1;

	}
	return 0;
}

// -------------------------------------------------------
// // 'GetFirmwareVersion()' - Get firmware version from motor
// // Passes in the Serial Port and baudrate
// // Returns the file descriptor on success or -1 on error.
// // Returns the firmare version in temp paramater
// // ----------------------------------------------------
int GetFirmwareVersion(int Port, int Baud, char* temp, char echo)
{
	char buf;
	int i;
	int j =0;
	char fware[8];
	long num,q,r;
	char hex[100];
	struct termios options;
	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -1;
    	}
	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSANOW, &options) <0)
		{

			return -1;
		}
	}

	while(read(Port,&buf, 1)>0)
	{
	}
	// Read firmware version from motor
	if(write(Port,"RFW ",4)==0)
	{

		return -2;
	}
	if(echo)
	{
		for (i =0; i <4; i++)
		{
			read(Port, &buf, 1);
		}
	}
	i=0;
	while(read(Port,&fware[i], 1)>0)
	{
		i++;
	}
	sscanf(fware, "%lu", &num);
	q = num;
	while( q != 0)
	{
		r = q % 16;
		if (r <10)
		{
			hex[j++] = 48 + r;
		}
		else
		{
			hex[j++] = 55 + r;
		}
		q = q / 16;
	}
	printf("Firmware Version (hex): ");
	sprintf(temp,"%c%c.%c%c.%c%c.%c%c",hex[7],hex[6],hex[5],hex[4],hex[3],hex[2]
			,hex[1],hex[0]);
	for (i=1;i<16;i++)
	{
		printf("%c",temp[i]);
	}
	puts(" ");

	return 0;
}

// -------------------------------------------------------
// // 'GetSerial()' - Get Serial Number  from motor
// // Passes in the Serial Port and baudrate
// // Returns the file descriptor on success or -1 on error.
// // Returns serial number in temp parameter
// // -----------------------------------------------------
int GetSerial(int Port, int Baud, char* temp, char echo)
{
	char buf;
	char hold[10];
	char hex[100];
	int i;
	int j=0;
	long dec,q,r;
	char head[2];
	char tail[8];
	char chr;
	int k =0;
	int l=0;
	unsigned int tmp2;
	unsigned int tmp;
	struct termios options;
	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -1;
    	}
	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSANOW, &options) <0)
		{

			return -1;
		}
	}

	while(read(Port,&buf, 1)>0)
	{
	}
	// Set EE pointer to Serial Number, store the value in variable a and read a from motor
	if(write(Port,"EPTR=32512 VLD(a,4) Ra ",23)==0)
	{
		return -2;
	}
	if(echo)
	{
		for(i =0; i < 23; i++)
		{
			read(Port,(unsigned char*)&buf, 1);
		}
	}
	i =0;
	while(read(Port,(unsigned char*)&buf,1)>0)
	{
		hold[i] = buf;
		i++;
	}
	i=0;

	sscanf(hold,"%lu", &dec);
	if(dec <= 0)
	{
		printf("No Serial Number");
		return -100;
	}
	// dec to hex
	q = dec;
	while( q != 0)
	{
		r = q % 16;
		if (r <10)
		{
			hex[j++] = 48 + r;
		}
		else
		{
			hex[j++] = 55 + r;
		}
		q = q / 16;
	}
	for(i=(j-1); i>=4;i--)
	{
		head[0] = hex[i-k];
		head[1] = hex[i-k-1];

		if(i ==(j-1))
		{
			sscanf(head,"%04x",&tmp2);
			chr = (char) tmp2;
		}
		else
		{
			tail[l] = head[0];
			tail[l+1]= head[1];
			l=l+2;
		}
		k ++;

	}
	sscanf(tail,"%16x",&tmp);
	long unsigned int tmp1 = tmp;
	sprintf(temp,"%c%lu\n",tmp2, tmp1);
	printf("Serial Number: %c%lu\n", tmp2,tmp1);

	return 0;
}

// -------------------------------------------------------
// // Passes in the Serial Port and baudrate
// // Returns the file descriptor on success or -1 on error.
// // Returns the date in temp parameter
// // -----------------------------------------------------
int GetProdDate(int Port, int Baud, char* temp, char echo)
{
	char buf,x;
	int i;
	int j =0;
	long dec,q,r;
	char hold[100];
	char hex[100];
	char year[2];
	char month[2];
	char day[2];
	int y,m,d;
	unsigned int  tmp;
	struct termios options;

	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -1;
    	}
	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSANOW, &options) <0)
		{

			return -1;
		}
	}

	while(read(Port,&buf, 1)>0)
	{
	}

	// Set EE Pointer, store value in a and read a from motor
	if(write(Port,"EPTR=32539 VLD(a,3) Ra ",23)==0)
	{
		return -2;
	}
	if(echo)
	{
		for(i =0; i < 23; i++)
		{
			read(Port,(unsigned char*)&buf, 1);
		}
	}
	i=0;
	while(read(Port,(unsigned char*)&x,1) >0)
	{
		hold[i]= x;
		i++;
	}
	sscanf(hold, "%lu", &dec);
	// dec to hex
	q = dec;
	i=0;
	while( q != 0)
	{
		r = q % 16;
		if (r <10)
		{
			hex[j++] = 48 + r;
		}
		else
		{
			hex[j++] = 55 + r;
		}
		q = q / 16;
		i++;
	}

	if( i == 5)
	{

		year[0]  = hex[j-1];
		month[0] = hex[j-2];
		month[1] = hex[j-3];
		day[0]	 = hex[j-4];
		day[1] 	 = hex[j-5];
	}
	else if (i == 6)
	{
		year[0]  = hex[j-1];
		year[1]  = hex[j-2];
		month[0] = hex[j-3];
		month[1] = hex[j-4];
		day[0]	 = hex[j-5];
		day[1] 	 = hex[j-6];
	}
	else
	{
		puts("Motor Echo is on, pass in 1 for echo param");
		return -6;
	}
	sscanf(year, "%2x", &tmp);
	y = (int) tmp;
	sscanf(month,"%2x", &tmp);
	m = (int) tmp;
	sscanf(day,"%2x", &tmp);
	d = (int) tmp;

	if( m == 255)
	{
		// not in prod
		// hasnt shipped
		temp = "N/A";
	}
	else
	{
		sprintf(temp,"Animatics Production date: %d/%d/20%d\n", m,d,y);
	}
	printf("%s\n", temp);
	return 0;
}

// -------------------------------------------------------
// // 'GetValue()' - Read value from motor
// // Passes in the Serial Port, command( RVT, RTEMP etc) and baudrate
// // Returns the file descriptor on success or -1 on error.
// // Returns value in the value parameter
// // -----------------------------------------------------

int GetValue(int Port, int Baud, char* Command, long value, char echo, char* MotorNo)
{
	char buf;
	int i;
	struct termios options;

	write(Port, (unsigned char*)&MotorNo, 1);

	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -1;
    	}
	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSANOW, &options) <0)
		{

			return -1;
		}
	}

	while(read(Port,&buf, 1)>0)
	{
	}
	if(write(Port,Command, sizeof(Command))==0)
	{
		return -2;
	}
	if(echo)
	{
		for (i =0; i < sizeof(Command);i++)
		{
			read(Port,(unsigned char*)&buf, 1);
		}
	}
	int count = 0;
	char hold [20];

	while(read(Port,(unsigned char*)&buf,1) > 0)
	{
		hold[count] = buf;
		count ++;
	}
	sscanf(hold, "%ld", &value);
	printf("%s%ld\n", "value: " , value);

	return value;
}

//test function for setting diff baudrates
int testBaud(int Port, int Baud)
{
	int bps[5] = {9600,19200,38400,57600,115200};
	struct termios options;

	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSAFLUSH, &options) <0)
		{

			return -1;
		}
	}
	if( SetBaudrate(Port,bps[1]) != 0)
    	{
		puts("error setting baudrate");
		return -1;
    	}

	int i;
	char a= 0x80;
	write(Port,(unsigned char*)&a, 1);
	if( write(Port,"WAKE ",5)==0)
	{
		return -1;
	}

	for(i=4; i>-1;i--)
	{
//		sleep(5);
		if( SetBaudrate(Port,bps[i]) != 0)
    		{
			puts("error setting baudrate");
			return -2;
   		}

		write(Port,(unsigned char*)&a, 1);
		if( write(Port,"WAKE ",5)==0)
		{
			return -1;
		}

	}
	return 0;
}

int EstLinkNew(int Port, int Baud)
{
	char buf, gotwake,echo;
	char a = 0x80;
	int i;
	int MaxMotors;
//	int MotorsFound = 0;
	int SerialBitRates[5] = {9600,19200,38400,57600,115200};
	struct termios options;

	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -1;
    	}
	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSAFLUSH, &options) <0)
		{

			return -1;
		}
	}
	echo = 0;
	gotwake = 0;
	MaxMotors = 10;
	int count, j;
	char message[14];
	char ochn[27];
	char found =0;
	// try at configured baudrate
	for (i=0; i < MaxMotors; i++)
	{
		count = 0;
		//purge
		while(read(Port,&buf, 1)>0)
		{
		}
		write(Port,(unsigned char*)&a, 1);
		if( write(Port,"WAKE ",5)==0)
		{
			return -1;
		}
		write(Port,(unsigned char*)&a, 1);
		if( write(Port,"WAKE ",5)==0)
		{
			return -1;
		}
		write(Port,(unsigned char*)&a, 1);
		if( write(Port,"ECHO ",5)==0)
		{
			return -1;
		}
		while(read(Port,(unsigned char*)&buf,1) >0)
		{
			if((count ==0)&& (buf == 'W'))
			{
				count ++;
			}
			else if((count ==1)&& (buf == 'A'))
			{
				count ++;
			}
			else if((count ==2)&& (buf == 'K'))
			{
				count ++;
			}
			else if((count ==3)&& (buf == 'E'))
			{
				count ++;
			}
			else if((count ==4) && (buf == ' '))
			{
				found =1;
				break;
			}
			else if(count !=0)
			{
				count ++;
			}
		}
		if(found == 1)
		{
			gotwake =1;
			found =0;

		}

	}

	if(gotwake)
	{
		while(read(Port, &buf,1) >0)
		{
		}
		echo = 1;
		gotwake = 0;
		found = 0;
		count =0;
		write(Port,"ECHO ",5);
		while(read(Port,(unsigned char*)&buf,1) >0)
		{
			if((count ==0)&& (buf == 'E'))
			{
				count ++;
			}
			else if((count ==1)&& (buf == 'C'))
			{
				count ++;
			}
			else if((count ==2)&& (buf == 'H'))
			{
				count ++;
			}
			else if((count ==3)&& (buf == 'O'))
			{
				count ++;
			}
			else if((count ==4) && (buf == ' '))
			{
				echo = 0;
				break;
			}
			else if(count !=0)
			{
				count ++;
			}
		}
		if(echo ==0)
		{
			return 0;

		}
		else
		{
			return -2;
		}
	}
	else
	{
		// work down from 38400 first
		for(i=0; i< MaxMotors; i++)
		{	count = 0;
			found =0;
			for(j=2; j> -1; j--)
			{


				sprintf(message,"BAUD%d ", SerialBitRates[j]);
				write(Port,(unsigned char*)&a, 1);
				write(Port, message, 10);
				write(Port,(unsigned char*)&a, 1);
				if( write(Port,"WAKE ",5)==0)
				{
					return -1;
				}
				write(Port,(unsigned char*)&a, 1);
				if( write(Port,"WAKE ",5)==0)
				{
					return -1;
				}
				if( SetBaudrate(Port,SerialBitRates[j]) != 0)
    				{
					puts("error setting baudrate");
					return -1;
    				}


			}

			write(Port,(unsigned char*)&a, 1);
			if( write(Port,"WAKE ",5)==0)
			{
				return -1;
			}
			write(Port,(unsigned char*)&a, 1);
			if( write(Port,"WAKE ",5)==0)
			{
				return -1;
			}
			write(Port,(unsigned char*)&a, 1);
			if( write(Port,"ECHO ",5)==0)
			{
				return -1;
			}
			count = 0;
			found = 0;
			while(read(Port,(unsigned char*)&buf,1) >0)
			{
				if((count ==0)&& (buf == 'W'))
				{
					count ++;
				}
				else if((count ==1)&& (buf == 'A'))
				{
					count ++;
				}
				else if((count ==2)&& (buf == 'K'))
				{
					count ++;
				}
				else if((count ==3)&& (buf == 'E'))
				{
					count ++;
				}
				else if((count ==4) && (buf == ' '))
				{
					found =1;
					break;
				}
				else if(count !=0)
				{
					count ++;
				}
			}
			if(found)
			{

				gotwake= 1;

			}


		}


		if(gotwake)
		{
			//original baud rate

			sprintf(message,"BAUD%d ", Baud);
			write(Port,(unsigned char*)&a,1);
			write(Port, message,10);
			if( SetBaudrate(Port,Baud) != 0)
    			{
				puts("error setting baudrate");
				return -1;
    			}
			sleep(.2);
			sprintf(ochn,"OCHN(RS2,0,N,%d,1,8,C) ",Baud);
			write(Port,(unsigned char*)&a,1);
			write(Port,ochn,27);
			while(read(Port, &buf,1) >0)
			{
			}
			echo = 1;
		}


	}
	if(gotwake)
	{
		gotwake =0;
		count =0;
		while(read(Port, (unsigned char*)&buf,1)>0)
		{
		}
		if(write(Port,"ECHO ",5)==0)
		{
			return -2;
		}
		while(read(Port,(unsigned char*)&buf,1) >0)
		{
			if((count ==0)&& (buf == 'E'))
			{
				count ++;
			}
			else if((count ==1)&& (buf == 'C'))
			{
				count ++;
			}
			else if((count ==2)&& (buf == 'H'))
			{
				count ++;
			}
			else if((count ==3)&& (buf == 'O'))
			{
				count ++;
			}
			else if((count ==4) && (buf == ' '))
			{
				echo = 0;
				break;
			}
			else if(count !=0)
			{
				count ++;
			}

		}
		if(echo ==0)
		{
			return 0;

		}
		else
		{
			return -2;
		}
	}
	// 115200 and 57600
	else
	{
		echo =0;
		for(i=0;i < MaxMotors; i++)
		{
			for(j =4; j>2;j--)
			{
				sprintf(message,"BAUD%d ", SerialBitRates[j]);
				write(Port,(unsigned char*)&a, 1);
				write(Port, message, 10);
				write(Port,(unsigned char*)&a, 1);
				if( write(Port,"WAKE ",5)==0)
				{
					return -1;
				}
				write(Port,(unsigned char*)&a, 1);
				if( write(Port,"WAKE ",5)==0)
				{
					return -1;
				}
				if( SetBaudrate(Port,SerialBitRates[j]) != 0)
    				{
					puts("error setting baudrate");
					return -1;
    				}
			}

			write(Port,(unsigned char*)&a, 1);
			if( write(Port,"WAKE ",5)==0)
			{
				return -1;
			}
			write(Port,(unsigned char*)&a, 1);
			if( write(Port,"WAKE ",5)==0)
			{
				return -1;
			}
			write(Port,(unsigned char*)&a, 1);
			if( write(Port,"ECHO ",5)==0)
			{
				return -1;
			}
			count = 0;
			found = 0;
			while(read(Port,(unsigned char*)&buf,1) >0)
			{
				if((count ==0)&& (buf == 'W'))
				{
					count ++;
				}
				else if((count ==1)&& (buf == 'A'))
				{
					count ++;
				}
				else if((count ==2)&& (buf == 'K'))
				{
					count ++;
				}
				else if((count ==3)&& (buf == 'E'))
				{
					count ++;
				}
				else if((count ==4) && (buf == ' '))
				{
					found =1;
					break;
				}
				else if(count !=0)
				{
					count ++;
				}
			}
			if(found)
			{
				gotwake= 1;
			}
		}

		// last option in loop just try it
		sprintf(message,"BAUD%d ", Baud);
		write(Port,(unsigned char*)&a,1);
		write(Port, message,10);
		if( SetBaudrate(Port,Baud) != 0)
    		{
			puts("error setting baudrate");
			return -1;
    		}
		sleep(.2);
		sprintf(ochn,"OCHN(RS2,0,N,%d,1,8,C) ",Baud);
		write(Port,(unsigned char*)&a,1);
		write(Port,ochn,27);
		while(read(Port, &buf,1) >0)
		{
		}
		echo = 1;
		count =0;
		while(read(Port, (unsigned char*)&buf,1)>0)
		{
		}
		if(write(Port,"ECHO ",5)==0)
		{
			return -2;
		}
		while(read(Port,(unsigned char*)&buf,1) >0)
		{
			if((count ==0)&& (buf == 'E'))
			{
				count ++;
			}
			else if((count ==1)&& (buf == 'C'))
			{
				count ++;
			}
			else if((count ==2)&& (buf == 'H'))
			{
				count ++;
			}
			else if((count ==3)&& (buf == 'O'))
			{
				count ++;
			}
			else if((count ==4) && (buf == ' '))
			{
				echo = 0;
				break;
			}
			else if(count !=0)
			{
				count ++;
			}
		}
		if(echo ==0)
		{
			return 0;

		}
		else
		{
			return -2;
		}

	}
	return 0;
}

// -------------------------------------------------------
// // 'DetectRS23()'
// // Detect motors on RS232 chain
// // Passes in the Serial Port and baudrate
// // Returns -1 on error.
// // Return value based on what is detected
// // Does not detect Baudrates 4800 and below
// // ----------------------------------------------------
int DetectRS232(int Port, int Baud)
{
	int retval= -1;
	int MaxMotors=10;
	int i;
	char a = 0x80;
	char f1 = 0xf2;
	char addr_t[8];
	char resp =0;
	char buf;
	int MotorsFound = 0;
	int lastaddr =0;
	retval = EstLinkNew(Port, Baud);
	if(retval <0)
	{

		return -1;
	}
	write(Port,(unsigned char*)&a,1);
	write(Port,"WAKE ",5);
	write(Port,(unsigned char*)&a,1);
	write(Port,"WAKE ",5);

	//+1 for addr 114
	for (i=1; i <=MaxMotors+ 1;i++)
	{
		a++;
		resp =0;
		if(i == MaxMotors+1)
		{
			// write to address 114
			while(read(Port,(unsigned char*)&buf,1)>0)
			{
			}
			write(Port,(unsigned char*)&f1,1);
			write(Port,"RBe ",4);
			while( read(Port,(unsigned char*)&buf, 1) >0)
			{
				if( buf == '0' || buf == '1')
				{
					resp =1;
				}
			}
			if(resp ==1)
			{
				lastaddr++;
				sprintf(addr_t,"ADDR=%d ",lastaddr);
				write(Port,(unsigned char*)&f1,1);
				write(Port,addr_t,8);
			}

		}
		else
		{
			write(Port,(unsigned char*)&a,1);
			write(Port,"RBe ",4);
			while( read(Port,(unsigned char*)&buf, 1) >0)
			{
				if( buf == '0' || buf == '1')
				{
					resp =1;
				}
			}
			if(resp ==1)
			{
				MotorsFound++;
			}
			lastaddr=i;





		}
	}

	//Sleep detected motors
	a = 0x80;
	resp =0;
	for(i=0; i < lastaddr ;i ++)
	{
		a++;
		write(Port,(unsigned char*)&a,1);
		write(Port,"SLEEP ",6);
	}
	char MoreMotors = 1;
	a= 0x80;
	while(read(Port,(unsigned char*)&buf,1)>0)
		{
		}
	write(Port,(unsigned char*)&a,1);
	write(Port, "RBe ",4);
	while( read(Port,(unsigned char*)&buf, 1) >0)
	{

		if( buf == '0' || buf == '1')
		{
			MotorsFound++;
			resp =1;
		}
	}
	if(resp==0)
	{
		MoreMotors =0;
	}

	char Unaddressed = 1;
	resp =0;
	if(MoreMotors)
	{
		for (i =MaxMotors+1; i <= 127; i++)
		{
			a = a +1;
			write(Port,(unsigned char*)&a, 1);
			write(Port,"SLEEP ",6);
			a = 0x80;
		}
		while(read(Port,(unsigned char*)&buf,1)>0)
		{
		}
		write(Port,(unsigned char*)&a,1);
		write(Port, "RBe ",4);
		while( read(Port,(unsigned char*)&buf, 1) >0)
		{
			if( buf == '0' || buf == '1')
			{
				resp =1;
			}
		}
		if(resp ==1)
		{
			Unaddressed = 0;;
		}
	}

	write(Port,(unsigned char*)&a,1);
	write(Port,"WAKE ",5);

	if(MoreMotors)
	{
		if(Unaddressed)
		{
			if(MotorsFound > 0)
			{
				return 2;
			}
			else
			{
				return 16;
			}
		}
		else
		{
			return 4;
		}
	}

	return 1;
}

// -------------------------------------------------------
// // 'AddressRS23()'
// // addresses motors on RS232 chain
// // Passes in the Serial Port and baudrate
// // Returns -1 on error.
// // Does not detect Baudrates 4800 and below
// // ----------------------------------------------------

int AddressRS232(int Port, int Baud)
{
	int MaxMotors = 10;
	int val = -1;
	char a = 0x80;
	val = DetectRS232(Port,Baud);
	if(val <0)
	{
		puts("No Motors Detected");
		return -1;
	}
	else
	{
		if(val == 1) puts("Motors Found"); // Motors Found
		if(val == 2) puts("Some Motors Addressed"); // Some Motors addresed
		if(val == 4) puts("Some Motor address bigger than max"); // Address of some motors is bigger than Max address
		if(val == 8) puts("Duplicate address"); // Duplicate address found
		if(val == 16) puts("No Motor addressed");;// No motor addressed
	}



	write(Port,(unsigned char*)&a,1);
	write(Port,"END ",4);
	write(Port,(unsigned char*)&a,1);
	write(Port,"SADDR0 ",7);
	write(Port,(unsigned char*)&a,1);
	write(Port,"ECHO_OFF ",9);

	int MotorNo =0;
	int MotorAddr;
	char cont = 1;
	char saddr[9];
	char b = a;
	char buf;
	while(cont)
	{
		MotorAddr= MotorNo + 1;
		b++;
		sprintf(saddr,"SADDR%d ",MotorAddr);
		write(Port,(unsigned char*)&a,1);
		write(Port,saddr,9);

		write(Port,(unsigned char*)&b,1);
		write(Port,"ECHO ",5);
		write(Port,(unsigned char*)&b,1);
		write(Port,"SLEEP ",6);
		read(Port,(unsigned char*)&buf,1);

		MotorNo ++;

		if(MotorNo >= MaxMotors)
		{
			cont =0;
		}



	}
	MotorNo++;

	write(Port,(unsigned char*)&a,1);
	write(Port,"WAKE ",5);
	// to check addressing
	write(Port,(unsigned char*)&a,1);
	write(Port,"RADDR ",6);

	return 0;
}

// -------------------------------------------------------
// // 'DetectRS485()'
// // Detects Motors across RS485
// // Passes in the Serial Port and baudrate
// // Returns -1 on error.
// // Does not detect Baudrates 4800 and below
// // ----------------------------------------------------
int DetectRS485(int Port, int Baud)
{
	char a = 0x80;
	char f = 0xf2;
	char addr[9];
	char buf,resp;
	int i;
	int MaxMotors =10;
	int lastaddr =0;
	int MotorsFound = 0;
	struct termios options;

	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -1;
    	}
	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSAFLUSH, &options) <0)
		{

			return -1;
		}
	}
	write(Port,(unsigned char*)&a,1);
	write(Port,"WAKE1 ",6);
	write(Port,(unsigned char*)&a,1);
	write(Port,"WAKE1 ",6);
	write(Port,(unsigned char*)&a,1);
	write(Port,"IFX ",4);



	for(i=1;i<=MaxMotors+1;i++)
	{
		a++;
		resp = 0;
		if(i== MaxMotors+1)
		{
			while(read(Port,(unsigned char*)&buf,1)>0)
			{
			}
			write(Port,(unsigned char*)&a,1);
			write(Port,"RBe ",4);
			while( read(Port,(unsigned char*)&buf, 1) >0)
			{
				if( buf == '0' || buf == '1')
				{
					resp=1;
				}
			}
			if(resp ==1)
			{
				lastaddr++;
				sprintf(addr,"ADDR=%d ",lastaddr);
				write(Port,(unsigned char*)&f,1);
				write(Port,addr,9);
				resp =0;
			}
		}
		else
		{
			while(read(Port,(unsigned char*)&buf,1)>0)
			{
			}
			write(Port,(unsigned char*)&a,1);
			write(Port,"RBe ",4);

			while( read(Port,(unsigned char*)&buf, 1) >0)
			{
				if( buf == '0' || buf == '1')
				{
					resp =1;
				}
			}

			if(resp ==1)
			{
				MotorsFound++;
			}
			lastaddr =i;
		}

	}
	a=0x80;
	//sleep detected motors only
	for(i=0; i<lastaddr;i++)
	{
		a++;
		write(Port,(unsigned char*)&a,1);
		write(Port,"SLEEP1 ",7);

	}
	a=0x80;
	resp =0;
	char MoreMotors = 1;
	while(read(Port,(unsigned char*)&buf,1)>0)
	{
	}
	write(Port,(unsigned char*)&a,1);
	write(Port,"RBe ",4);
	while( read(Port,(unsigned char*)&buf, 1) >0)
	{
		if( buf == '0' || buf == '1')
		{
			MotorsFound++;
			resp=1;
		}
	}
	if(resp ==0)
	{
		MoreMotors =0;
	}

	char Unaddressed =1;
	resp =0;
	if(MoreMotors)
	{
		for(i=MaxMotors+1;i<= 127;i++)
		{
			a = a+i;
			write(Port,(unsigned char*)&a,1);
			write(Port,"SLEEP1 ",7);
			a=0x80;
		}
		while(read(Port,(unsigned char*)&buf,1)>0)
		{
		}
		write(Port,(unsigned char*)&a,1);
		write(Port,"RBe ",4);
		while( read(Port,(unsigned char*)&buf, 1) >0)
		{
			if( buf == '0' || buf == '1')
			{
				resp =1;
			}
		}
		if(resp ==0)
		{
			Unaddressed = 0;;
		}
	}
	write(Port,(unsigned char*)&a,1);
	write(Port,"WAKE1 ",6);

	if(MoreMotors)
	{
		if(Unaddressed)
		{
			if(MotorsFound > 0)
			{
				puts("Some Motors are addressed, some are not");
				return 2;
			}
			else
			{
				puts("No Motor has address");
				return 16;
			}
		}
		else
		{
			puts("Some Motors have addr bigger than Max");
			return 4;
		}


	}

	return 0;
}

int DriveReady(int Port, int Baud)
{
	char buf;

	struct termios options;

	if( SetBaudrate(Port,Baud) != 0)
    	{
		puts("error setting baudrate");
		return -1;
    	}
	if(tcgetattr(Port, &options) < 0)
	{
		return -1;
	}
	if ((options.c_cc[VMIN] != 0) || (options.c_cc[VTIME] != 10))
	{
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 10;
		if(tcsetattr(Port, TCSANOW, &options) <0)
		{

			return -1;
		}
	}

	while(read(Port,&buf, 1)>0)
	{
	}

	char B00,IN7;
	char Ba,IN12,Bh;
	char Be,B03,Bv;
	char B09,Br,Bl;
	char Bten,Brs,Bls;
	char B28,B29,B65;
	char B67,B612, Bsix;
	char FWtemp, Beleven;
	char FWarray[16];
	long FW;
	int i =0;
	write(Port,"ECHO_OFF ",9);
	write(Port,"RB(0,0) ",8);
	read(Port,(unsigned char*)&B00,1);
	write(Port,"RB(6,13) ",9);
	read(Port,(unsigned char*)&Bsix,1);
	write(Port,"RFW ",4);
	while((read(Port,(unsigned char*)&FWtemp,1)>0))
	{
		FWarray[i] = FWtemp;
		i++;

	}
	sscanf(FWarray,"%lu",&FW);

	write(Port,"RIN(7) ",7);
	read(Port,(unsigned char*)&IN7,1);
	write(Port,"RBa ",4);
	read(Port,(unsigned char*)&Ba,1);
	write(Port,"RIN(12) ",8);
	read(Port,(unsigned char*)&IN12,1);
	write(Port,"RBh ",4);
	read(Port,(unsigned char*)&Bh,1);
	write(Port,"RBe ",4);
	read(Port,(unsigned char*)&Be,1);
	write(Port,"RB(0,3) ",8);
	read(Port,(unsigned char*)&B03,1);
	write(Port,"RBv ",4);
	read(Port,(unsigned char*)&Bv,1);
	write(Port,"RB(0,9) ",8);
	read(Port,(unsigned char*)&B09,1);
	write(Port,"RBr ",4);
	read(Port,(unsigned char*)&Br,1);
	write(Port,"RBl ",4);
	read(Port,(unsigned char*)&Bl,1);
	write(Port,"RB(1,10) ",9);
	read(Port,(unsigned char*)&Bten,1);
	write(Port,"RB(1,11) ",9);
	read(Port,(unsigned char*)&Beleven,1);
	write(Port,"RBrs ",5);
	read(Port,(unsigned char*)&Brs,1);
	write(Port,"RBls ",5);
	read(Port,(unsigned char*)&Bls,1);
	write(Port,"RB(2,8) ",8);
	read(Port,(unsigned char*)&B28,1);
	write(Port,"RB(2,9) ",8);
	read(Port,(unsigned char*)&B29,1);
	write(Port,"RB(6,5) ",8);
	read(Port,(unsigned char*)&B65,1);
	write(Port,"RB(6,7) ",8);
	read(Port,(unsigned char*)&B67,1);
	write(Port,"RB(6,12) ",9);
	read(Port,(unsigned char*)&B612,1);


	///////////////////////////////////
	// first look at low voltage and drive enable input which can prevent drive ready
	if(B00 != '0')
	{
		// Clear drive voltage low flag to see is condition is still true
		write(Port,"Z(6,13) ",8);
		sleep(.01);
		if(Bsix == '1') //not a fault but will prevent drive ready
		{
			printf("Bus voltage is too low\n");
		}
		if((FW/(2^24)) == 6)	// Class 6
		{
			if(IN7 =='0')
			{
				printf("Drive enable input low\n");
			}
			if(Ba == '1')
			{
				printf("Overcurrent occurred\n");
			}
		}

		else if((FW/(2^24))==5) // Class 5
		{
			if((((FW/65536)&255) == 98) | (((FW/65536)&255) == 97))
			// 5M-Series 5.98.x.x or 5.97.x.x
			{
				if(IN12 == '0')
				{
					printf("Drive enable input low\n");
				}
				if(Ba == '1')
				{
					printf("Overcurrent occured\n");
				}
			}
		}
		//////////////////////////////
		// Now look at faults

		if(Bh == '1')
		{
			printf("Over Temperature\n");
		}
		if(Be =='1')
		{
			printf("Excessive Position Error\n");
		}
		if(B03=='1')
		{
			printf("Bus Voltage Faule\n");
		}
		if(Bv =='1')
		{
			printf("Velocity Limit Fault\n");
		}
		if(B09=='1')
		{
			printf("DE/DT Fault\n");
		}
		if(Br=='1')
		{
			printf("Historical Positive H/W Limit\n");
		}
		if(Bl=='1')
		{
			printf("Historical Negative H/W Limit\n");
		}
		if(Bten=='1' && Beleven=='1')
		// Software Limits enabled and mode to cause faults
		{
			if(Brs =='1')
			{
				printf("Historical Positive S/W Limit Fault\n");
			}
			if(Bls =='1')
			{
				printf("Historical Negative S/W Limit Fault\n");
			}
		}
		if(B28=='1')
		{
			printf("Watchdog\n"); //NOTE: not all firmware supports this, but if
					      //this bit is set then positevely know that is true
		}
		if(B29=='1')
		{
			printf("ADB checksum Fault\n"); // ADB is corrupt, prevent motion
		}
		if(B65=='1')
		{
			printf("Feedback Fault (encoder)\n");
		}
		if(B67=='1')
		{
			printf("Drive Enable Faut\n");
		}
		if(B612=='1')
		{
			printf("ABS Battery Fault\n"); // NOTE: there is a way to make this bit not fault the motor
							// in which case this statement would still print
		}
	}
	else
	{
		printf("Drive is Ready!\n");
	}


	return 0;
}
