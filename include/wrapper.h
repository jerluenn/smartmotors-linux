#include <chrono>

class SmartMotors {

  public:

    int Port = 0;
    int Baud = 115200;
    int Base = 0x80;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_start;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_end;
    double elapsed_time_ms;

    SmartMotors();

    void command(char* line, int MotorNo);

    void commandAll(char* line);

    void read(char* line, int MotorNo);

    void exiting();

    void tic();

    void toc();

};
