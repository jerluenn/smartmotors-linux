class SmartMotors {

  public:

    int Port = 1;
    int Baud = 9600;
    int Base = 0x80;

    SmartMotors();

    void command(char* line, int MotorNo);

    void commandAll(char* line);

};
