#include "uart.h"

#include <iostream>
#include <string>
#include <unistd.h>

#include <cstring> //for strlen()


int main(){
    uart serialCom = uart(115200, "/dev/ttyACM3");
    char data[1024];


    char msg[] = "100 100 100 100 100 100 100 100\n";

    while(1){
        serialCom.write_data(msg, strlen(msg)); //size must be EXACTLY how many chars to send
        std::cout << "Wrote: " << msg;

        serialCom.read_data(data);
        for (int i = 0; i < 1023 && data[i] != '\0'; ++i) {
            std::cout << data[i];
        }


        usleep(1000000);
    }
}
