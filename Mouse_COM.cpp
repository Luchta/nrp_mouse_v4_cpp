#include "Mouse_COM.h"
//#include <stdio.h>
#include <unistd.h>	//Used for UART
#include <fcntl.h>	//Used for UART
#include <termios.h>	//Used for UART
#include <stdlib.h>
#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <iostream>

//Parsing defines for command lengths
#define MAX_RECIEVE_LENGTH 255
#define MAX_ARG_LENGTH 4
#define MAX_ARG_PER_LINE 4
//Defines for ID ranges
#define MIN_SERVO_ID 11
#define MAX_SERVO_ID 35
#define MIN_EVENT_ID 51
#define MAX_EVENT_ID 58
#define MIN_SENSOR_ID 61
#define MAX_SENSOR_ID 64
#define MIN_STREAM_ID 71
#define MAX_STREAM_ID 74

//Debugging Mode
#define DEBUG true

mouse_com::mouse_com()
{
    if(DEBUG){std::cout << "mouse_com init\n";}
    InitRPI();
    setup_uart_read();
    setup_uart_send();
}

mouse_com::~mouse_com()
{
    //----- CLOSE THE UART -----
    close(uart0_readstream);
    close(uart0_sendstream);
}

//starting the loop thread
void mouse_com::startThread() {
    std::cout << "starting UART and MOUSE thread"<<std::endl;
    t1 = std::thread ([=] { MouseInputLoop(); });
    t1.detach();
    std::cout << "Thread detached"<<std::endl;
}

//void mouse_com::setConsoleCcmnd(Ccmnd cmd){
void mouse_com::setConsoleCcmnd(typCmd cmd, int val1, int val2, int val3){
    Ccmnd tmp;

    tmp.command = cmd;
    tmp.val1 = val1;
    tmp.val2 = val2;
    tmp.val3 = val3;
    tmp.valid = true;

    //consoleCmnd.exchange(tmp);
    //consoleCmnd = cmd;
    consoleCmnd.store(tmp);
}


//Loop for recieving messages from UART
void mouse_com::MouseInputLoop()
{
    bool OK = true; //for later Break criteria

    while(OK)
    {

        //handle UART input
        recieveData();

        sleep(10); //waiting for data
    }
}

void mouse_com::setup_uart_send()
{
    //This code was taken from https://raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart

    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively


    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    uart0_sendstream = open("/dev/ttyAMA0", O_WRONLY | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    if (uart0_sendstream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        std::cerr << "Error - Unable to open UART.  Ensure it is not in use by another application!" << std::endl;
        //printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_sendstream, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_sendstream, TCIFLUSH);
    tcsetattr(uart0_sendstream, TCSANOW, &options);


}

void mouse_com::setup_uart_read()
{
    //This code was taken from https://raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart

    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively


    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    //uart0_readstream = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    uart0_readstream = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY);		//Open in blocking read mode

    if (uart0_readstream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        std::cerr << "Error - Unable to open UART.  Ensure it is not in use by another application!" << std::endl;
        //printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_readstream, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_readstream, TCIFLUSH);
    tcsetattr(uart0_readstream, TCSANOW, &options);


}

//UART-Schnittstelle zu CRPI
void mouse_com::ProcessSpine(mouse_com::typCmd cmd, int val1, int val2, int val3)
{
    switch (cmd)
    {
    case SetMotorPos:
        mouse_com::sendMotor_Serial(val1, val2, val3); //takes ID, Pos, Speed
        break;
    case GetSensorValue:
        mouse_com::sendSensorRequest(val1); //only takes ID
        break;
    default:
        //NOOOOOOO!
        std::cerr << "Error - Unknwon UART Send Request!" << std::endl;
        break;
    }

}

bool mouse_com::sendMotor_Serial(int id, int pos, int speed)
{
    //------ FORM STRING------
    int l;
    char buffer [18];
    l = sprintf (buffer, "%d %d %d", id, pos, speed);

    if (DEBUG){
        std::cout.write(&buffer[0], l);
    }

    //----- TX BYTES -----

    if (uart0_sendstream != -1)
    {
        int count = write(uart0_sendstream, &buffer[0], l);		//Filestream, bytes to write, number of bytes to write
        //int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0])); //Filestream, bytes to write, number of bytes to write
        if (count < 0)
        {
            //printf("UART TX error\n");
            std::cerr << "Error - UART TX ERROR" << std::endl;
            return false;
        }
    }
    else {
        //printf("Error: UART not opened\n");
        std::cerr << "Error - UART Sendstream not opened!" << std::endl;
        return false;
    }
    return true;
}

bool mouse_com::sendStreamRequest(int id, int frequency, int amount)
{
    //------ CONVERT TO HEX AND FORM STRING------
    int l;
    char buffer [18];
    l = sprintf (buffer, "%d %d %d", id, frequency, amount);

    //----- TX BYTES -----

    if (uart0_sendstream != -1)
    {
        int count = write(uart0_sendstream, &buffer[0], l);		//Filestream, bytes to write, number of bytes to write
        if (count < 0)
        {
            //printf("UART TX error\n");
            std::cerr << "Error - UART TX ERROR" << std::endl;
            return false;
        }
    }
    else {
        //printf("Error: UART not opened\n");
        std::cerr << "Error - UART Sendstream not opened!" << std::endl;
        return false;
    }
    return true;
}

bool mouse_com::sendSensorRequest(int id)
{
    //------ CONVERT TO HEX AND FORM STRING------
    int l;
    char buffer [18];
    l = sprintf (buffer, "%d", id);


    //----- TX BYTES -----

    if (uart0_sendstream != -1)
    {
        int count = write(uart0_sendstream, &buffer[0], l);		//Filestream, bytes to write, number of bytes to write
        if (count < 0)
        {
            //printf("UART TX error\n");
            std::cerr << "Error - UART TX ERROR" << std::endl;
            return false;
        }
    }
    else {
        //printf("Error: UART not opened\n");
        std::cerr << "Error: UART Sendstream not opened!" << std::endl;
        return false;
    }
    return true;
}

int mouse_com::recieveData()
{
    //parsing variables
    char const separator = ' ';
    char newArg[MAX_ARG_PER_LINE][MAX_ARG_LENGTH];
    int arguments[MAX_ARG_PER_LINE];
    int i = 0;
    int count = 0;
    int rx_length = 0;
    // Read up to 255 characters from the port if they are there
    char rx_buffer[MAX_RECIEVE_LENGTH+1];
    //int numArgs = 0;

    //----- CHECK FOR ANY RX BYTES -----
    if (uart0_readstream != -1)
    {
        while ((rx_length = read(uart0_readstream, (void*)rx_buffer, MAX_RECIEVE_LENGTH)) != 0)
        {
            //rx_length = read(uart0_readstream, (void*)rx_buffer, MAX_RECIEVE_LENGTH);		//Filestream, buffer to store in, number of bytes to read (max)
            if (rx_length < 0)
            {
                //An error occured (will occur if there are no bytes)
                std::cerr << "Error: UART Read returned no bytes!" << std::endl;
                return -1;
            }
            else if (rx_length == 0)
            {
                //No data waiting
                return 0;
            }
            else
            {
                //Bytes received
                rx_buffer[rx_length] = '\0';
                //DEBUGGIN-Comment OUT!
                printf("%i bytes read : %s\n", rx_length, rx_buffer);
                //parse to arguments
                //seperate string
                char *token = strtok(rx_buffer, &separator);
                while ((i < MAX_ARG_PER_LINE) && ((token = strtok(NULL, &separator)) != NULL))
                {
                    strcpy(newArg[i++], token);
                    count++;
                }
                //convert to int and store
                if (count == 0 || count < 0)
                {
                    //no Arguments read || Error - should never be reached
                }else {
                    for (i=0;i<count;i++) {
                        arguments[i] = atoi (newArg[i]);
                    }
                }
                //numArgs = count;

                //TODO PARSING BY ID WHICH MSG TYPE
                //check for
                if (arguments[0] >= MIN_SERVO_ID && arguments[0] <= MAX_SERVO_ID){
                    //motor IDs 11-14;21-24:31-35 (range 11-35)
                    ReceiveMsg(PosReached, arguments[0], arguments[1], 0);
                }else if (arguments[0] >= MIN_EVENT_ID && arguments[0] <= MAX_EVENT_ID) {
                    //event IDs 51-58

                }else if (arguments[0] >= MIN_SENSOR_ID && arguments[0] <= MAX_SENSOR_ID) {
                    //sensor IDs 61-64 (knees)
                    ReceiveMsg(SensorValue, arguments[0], arguments[1], 0);
                }else if (arguments[0] >= MIN_STREAM_ID && arguments[0] <= MIN_STREAM_ID) {
                    //stream IDs 71-74 (knees)
                }

                //return 1;
            }
            //check for Console Commands

            //load atomic struct
            Ccmnd tmp = consoleCmnd.load();
            //check for new commands
            if(tmp.valid)
            {
                tmp.valid = false;
                //consoleCmnd.store(tmp);
                ReceiveMsg((typCmd)tmp.command, tmp.val1, tmp.val2, tmp.val3);
            }
        }
        return 0;
    } else {
        std::cerr << "Error: UART Readstream not opened!" << std::endl;
    }
    return -1;
}

