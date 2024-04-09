#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

int main() {
    const char* serial_port = "/dev/ttyUSB0";
    //const char* serial_port = "/dev/ttyUSB1";
    //const char* serial_port = "/dev/bone/uart/1";
    int uart = open(serial_port, O_RDWR);
    //int uart = open(serial_port, O_RDONLY);
    if (uart == -1) {
        std::cerr << "Error: Unable to open serial port" << std::endl;
        return 1;
    }
    
    // Initialize UART
    struct termios options;
    tcgetattr(uart, &options);

    cfsetospeed(&options, B9600);
    cfsetispeed(&options, B9600);
    
    //options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    options.c_cflag &= ~PARENB;          // No parity bit
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~CSIZE;           // Clear data size bits
    options.c_cflag |= CS8;              // 8 data bits
    //options.c_cflag |= O_NONBLOCK;
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    
    tcsetattr(uart, TCSANOW, &options);

    std::cout << "LOLHEHEHE\n";
    while (true) {

        //Read incoming data
        uint8_t receivedData;
        if (read(uart, &receivedData, 1) == 1) {
            std::cout << "Received: " << static_cast<int>(receivedData) << std::endl;

            uint8_t ans = receivedData*2;
            std::cout << "Sending back: " << static_cast<int>(ans) << std::endl; 
            write(uart, &ans, 1);
            //uint8_t nl = 10;
            //write(uart, &nl, 1);
        }
        //usleep(100000);
    }

    close(uart);
    return 0;
}
