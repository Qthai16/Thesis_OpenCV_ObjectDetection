#include "../inc/raspi_uart.h"

PI_UART::PI_UART() {}

int PI_UART::uart_init(void)
{
    uart0_filestream = serialOpen ("/dev/serial0", BAUD_RATE);
    if (uart0_filestream < 0)
    {
	    printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	    std::cout << uart0_filestream << std::endl;
    }
    return uart0_filestream;
        // close(uart0_filestream);
}

int PI_UART::uart_string_send(uint8_t* data)
{
    int serial_flg_chk = -1;
    //serial_flg_chk = serialOpen ("/dev/serial0", BAUD_RATE);
    serial_flg_chk = uart_init();
    if (serial_flg_chk < 0)
    {
	    printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	    std::cout << serial_flg_chk << std::endl;
    }
    //else
    	//std::cout << serial_flg_chk << " success" << std::endl;
    if (serial_flg_chk > 0)
    {
        //while(*data != 0x00){
            serialPuts(serial_flg_chk, (char*)(data));
            serialFlush(serial_flg_chk);
            printf("%s", (char*)(data));
            fflush(stdout);
        //}
    }
    close(serial_flg_chk);
    return 0;
}

int PI_UART::uart_string_recv(uint8_t* data_recv)
{
    int index = 0;
    char recv_char;
    int serial_flg_chk= -1;
    serial_flg_chk = uart_init();
    if (serial_flg_chk > 0)
    {
        do{
            recv_char = serialGetchar(serial_flg_chk);
            printf("%c",recv_char);
	    *(data_recv+index) = recv_char;
	    index++;
            fflush (stdout);
        }
        while(serialDataAvail(serial_flg_chk));
    }
    return 0;
}

PI_UART::~PI_UART() {}
