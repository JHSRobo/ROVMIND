#include "uart.h"

#include <termios.h> //for UART
#include <fcntl.h>   //for UART
#include <unistd.h>  //for UART write/read
#include <sys/ioctl.h>
#include <iostream>
#include <cstdlib> // for std::exitS
#include <string>

uart::uart(int baud, const char* file)
{
    /*http://linuc.die.net/man/3/tcgetattr*/

    //Set user defined file name
    for (int i = 0; i < 19 && file[i] != '\0'; ++i) {
        m_fileName[i] = file[i];
        m_fileName[i+1] = '\0';
    }

    m_flags = O_RDWR | O_NOCTTY; //set flag to read/write non-blocking mode with no controlling terminal

    m_fd = open(m_fileName, m_flags);
    tcgetattr(m_fd, &m_tty);

    m_tty.c_cflag &= ~CSIZE;
    m_tty.c_cflag |= CS8; //set message size to 8bits

     //set baudrate
    switch(baud){
        case 9600:
            m_tty.c_cflag |= B9600;
            break;
        case 19200:
            m_tty.c_cflag |= B19200;
            break;
        case 38400:
            m_tty.c_cflag |= B38400;
            break;
        case 57600:
            m_tty.c_cflag |= B57600;
            break;
        case 115200:
            m_tty.c_cflag |= B115200;
            break;
        default:
            std::cout << "ERROR Invalid baudrate!\n";
            std::exit(1);
    }

    //set read and write cflag
    m_tty.c_cflag |= CREAD; //Enable receiver
    m_tty.c_cflag &= ~PARENB; //Enable even parity ->off
    m_tty.c_cflag &= ~CSTOPB; //No duplicate stop bits (protocol only uses one stop bit)

    m_tty.c_cflag &=  ~CRTSCTS; // no flow control

    //control modes lflag
    //Canonical mode -> makes input available at EOL
    m_tty.c_lflag |= ICANON;

    //input iflag

    //output oflag

    //commit the serial port settings
    tcflush(m_fd, TCIFLUSH); //flush port then apply attributes
    if(tcsetattr(m_fd, TCSANOW, &m_tty) != 0){
        std::cout << "ERROR Can not set " << uart::m_fd << " to correct settings\n";
        std::exit(2);
    }

}

uart::~uart()
{
    close(m_fd);
}


int uart::write_data(char* data, size_t dataSize){

    tcflush(m_fd, TCIFLUSH);
    m_fd = open(m_fileName, m_flags); //open the file
    if(write(m_fd, data, dataSize) != 0){
        close(m_fd);
        return -1;
    }
    close(m_fd);

    return 0;
}

int uart::read_data(char* buffer){

    tcflush(m_fd, TCIFLUSH);
    m_fd = open(m_fileName, m_flags); //open file for reading
    ssize_t length = read(m_fd, buffer, 1023);
    close(m_fd);

    return static_cast<int>(length);
}