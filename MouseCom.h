#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <thread>
#include <atomic>

//#include "RPI.h"

/*
class Ccmnd
{
public:
    Ccmnd(){}

    Ccmnd(int cmd, int v1=0, int v2=0, int v3=0){
        val1 = v1;
        val2 = v2;
        val3 = v3;
        command = cmd;
        valid = true;
    }

    bool valid = false;
    int val1, val2, val3;
    int command;
};
*/
struct Ccmnd
{
    bool valid = false;
    int val1 = 0, val2 = 0, val3 = 0;
    int command = 7; //FIXIT for TypCmd
};


class CMousCtrlSet;


class CMouseCom
{
public:
    CMouseCom();

    virtual ~CMouseCom();

    typedef enum Commmands{ SetMotorPos, PosReached, GetSensorValue, SensorValue,   // commands spine
                            MoveLeg, StepLeg, StepDone,                                // commands rpi internal
                            InitMouse, Trott, StopAll } typCmd;                        // commands from shell

    void startThread();

    void startUART();

    // for thread communication
    std::atomic<Ccmnd> consoleCmnd;

    //Ccmnd consoleCcmnd;

    //void setConsoleCcmnd(Ccmnd cmd);
    void setConsoleCcmnd(typCmd cmd, int val1=0, int val2=0, int val3=0);


protected:
    virtual void ProcessSpine(typCmd cmd, int val1, int val2, int val3);
    virtual CMousCtrlSet& InitRPI(){}
    virtual void ReceiveMsg(typCmd cmd, int val1=0, int val2=0, int val3=0) {}

private:

    std::thread t1; //waiting loop thread
    // UART STREAMS
    int uart0_readstream = -1;
    int uart0_sendstream = -1;
    //Variables
    //CRPI rpi;


    void setup_uart_send(); //sets up uart for read and write
    void setup_uart_read(); //sets up uart for read and write

    void MouseInputLoop();



    //sending funtions return true on success
    bool sendMotor_Serial(int id, int pos, int speed);
    bool sendStreamRequest(int id, int frequency, int amount);
    bool sendSensorRequest(int id);
    //recieve returns amount of arguments, 0 for nothing to read, -1 for error
    int recieveData(); // give array with MAX_ARG_LENGTH
    void checkComndConsole();
};



#endif // SERIAL_COM_H
