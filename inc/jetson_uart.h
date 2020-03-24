#ifndef JETSON_UART_H
#define JETSON_UART_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdlib>
#include <sys/signal.h>

// #define BAUD_RATE   9600

class JS_UART
{
public:
	JS_UART();
	~JS_UART();

	int jetson_uart_init(void); //call this function first
    //If the serial port is opened successfully, this function return 0; else return -1
    // int js_string_send(uint8_t* data_send);
    // int js_string_recv(uint8_t* data_recv);

    // void clean_and_exit(int code);
    // void quit_signal_handler(int signum);
    // void uart_signal_handler(int signum);

private:


};

#endif