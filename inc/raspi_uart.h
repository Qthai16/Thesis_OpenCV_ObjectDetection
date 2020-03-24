#ifndef RASPI_UART_H
#define RASPI_UART_H

#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <wiringPi.h>
#include <wiringSerial.h>

// gcc -Wall uart_send_test.c -o uart_test_2 -lwiringPi
#define BAUD_RATE   9600

class PI_UART
{
public:
	PI_UART();
	~PI_UART();
	int uart0_filestream = -1;
	int uart_init(void); //call this function first
    //If the serial port is opened successfully, this function return 0; else return -1
    int uart_string_send(uint8_t* data);
    int uart_string_recv(uint8_t* data_recv);

private:


};

#endif
