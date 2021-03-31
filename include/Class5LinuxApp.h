#ifndef Class5LinuxApp_h__
#define Class5LinuxApp_h__

int OpenPort(void);
void ClosePort(int Port);
int SetBaudrate(int Port, int Baud);
int DownloadSmxFile(int Port, int SMX, int Baud, char echo);
int DownloadSmxeFile(int Port, int SMXE, int Baud, char echo);
int DownloadFirmware(int Port, int Firmware, int Baud, char echo);
int UploadSMS(int Port, int Baud, char echo);
int UploadSMX(int Port, int Baud, char echo);
int WriteCommand(int Port, int Baud, char* FileName, char echo, char* MotorNo);
int GetFileName(int Port, char* Filename, char echo);
int GetFirmwareVersion(int Port, int Baud, char* temp, char echo);
int GetSerial(int Port, int Buad, char* temp, char echo);
int GetProdDate(int Port, int Baud, char* temp, char echo);
int GetValue(int Port, int Baud, char* Command, long value, char echo, char* MotorNo);
int testfunc(int Port);
int testBaud(int Port, int Baud);
int EstLinkNew(int Port, int Baud);
int DetectRS232(int Port, int Baud);
int AddressRS232(int Port, int Baud);
int DetectRS485(int Port, int Baud);
int DriveReady(int Port, int Baud);

#endif  // Class5LinuxApp_h__

