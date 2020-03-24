#include "../inc/jetson_uart.h"

static struct termios oldt, newt;

JS_UART::JS_UART() {}
JS_UART::~JS_UART() {}

extern "C" void quit_signal_handler(int signum);
extern "C" void uart_signal_handler(int signum);
extern "C" void clean_and_exit(int code);

int serial_descriptor = -1;

int JS_UART::jetson_uart_init(void)
{
    int c;

    // tty settings
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    // Canonical input (erase and kill processing), Enable echo
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);

    // Open serial port
    serial_descriptor = open("/dev/ttyTHS1", O_RDWR | O_NDELAY | O_NONBLOCK);
    // Open for reading and writing, non delay, non block
    if (serial_descriptor == -1)
    {
        printf("Could not open serial port on /dev/ttyTHS1!\n");
        // clean_and_exit(-1);
    }
    else fcntl(serial_descriptor, F_SETFL, 0);

    // Configure serial port
    struct sigaction saio;
    saio.sa_handler = uart_signal_handler;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO,&saio,NULL);

    fcntl(serial_descriptor, F_SETFL, FNDELAY);         // Make fd wait
    fcntl(serial_descriptor, F_SETOWN, getpid());       // Allow to receive SIGIO
    fcntl(serial_descriptor, F_SETFL,  O_ASYNC );       // Make fd asynchronous

    // UART settings
    struct termios termAttr;
    tcgetattr(serial_descriptor,&termAttr);
    cfsetispeed(&termAttr,B9600);                       // Input speed
    cfsetospeed(&termAttr,B9600);                       // Output speed
    termAttr.c_iflag = 0;                               // Turn off input processing
    termAttr.c_oflag = 1;                               // Turn of output processing
    termAttr.c_lflag = 0;                               // Turn off line procesinng
    termAttr.c_cflag = 0;                               // Turn off character processing
    termAttr.c_cflag |= (CS8|CREAD|CLOCAL);             // Read 8 bit
    termAttr.c_cc[VMIN] = 0;                            // No minimal chars
    // termAttr.c_cc[VTIME] = 1;                           // Wait 0.1 s for input
    tcsetattr(serial_descriptor,TCSANOW,&termAttr);   // Save settings
    /* Flush anything already in the serial buffer */
    tcflush(serial_descriptor, TCIFLUSH);
    tcflush(STDIN_FILENO, TCIFLUSH);
    //Kill signal handler
    signal(SIGINT,quit_signal_handler);

    return serial_descriptor;
}

void clean_and_exit(int code)
{
    // close serial
    close(serial_descriptor);
    // tty reset settings
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    // exit
    std::exit(code);
}

void quit_signal_handler(int signum)
{
    printf("Will end now!\n");
    clean_and_exit(0);
}

void uart_signal_handler(int signum)
{
    // char buffer[1024];
    // memset(buffer, '\0', 1024);
    // if ( ( read( serial_descriptor, &buffer, sizeof(buffer) ) ) > 0)
    /**{
		if (buffer[0] == 'q')
        {
            printf("Will end now!\n");
            clean_and_exit(0);
        } 
        else printf("Got command = %c\n", buffer[0]);
    }**/
}


