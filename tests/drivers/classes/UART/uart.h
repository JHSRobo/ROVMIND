#ifndef UART_H
#define UART_H

#include <termios.h> //for UART
#include <fcntl.h>   //for UART
#include <unistd.h>  //for UART
#include <string>

class uart
{

 struct termios m_tty;
 char m_fileName[20];
 int m_flags;
 int m_fd; //linux file identifier

 public:
    uart(int baud = 115200, const char* file = "/dev/ttyACM0"); //file is a max of twenty chars and must be null terminated
    virtual ~uart();

    //read all chars from the buffer
    int read_data(char*);

    int write_data(char*, size_t); //give a null terminated char array, returns -1 if an error occurred


};

#endif // UART_H
