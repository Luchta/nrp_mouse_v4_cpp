#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <thread>


#include "RPI.h"

class mouse_com
{
public:
    mouse_com();
    ~ mouse_com();

    void startThread();

#define MAX_RECIEVE_LENGTH 255
#define MAX_ARG_LENGTH 4
#define MAX_ARG_PER_LINE 4

private:
    std::thread t1; //waiting loop thread
    // UART STREAMS
    int uart0_readstream = -1;
    int uart0_sendstream = -1;
    //Variables
    CRPI rpi;

    void setup_uart_send(); //sets up uart for read and write
    void setup_uart_read(); //sets up uart for read and write

    void MouseInputLoop();

    //sending funtions return true on success
    bool sendMotor_Serial(int id, int pos, int speed);
    bool sendStreamRequest(int id, int frequency, int amount);
    bool sendSensorRequest(int id);
    //recieve returns amount of arguments, 0 for nothing to read, -1 for error
    int recieveData(); // give array with MAX_ARG_LENGTH
};


#endif // SERIAL_COM_H
