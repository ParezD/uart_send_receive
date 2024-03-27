#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

int main() {
    //const char* serial_port = "/dev/ttyUSB0";
    const char* serial_port = "/dev/ttyUSB0";
    int uart = open(serial_port, O_RDWR);
    //int uart = open(serial_port, O_WRONLY);
    if (uart == -1) {
        std::cerr << "Error: Unable to open serial port" << std::endl;
        return 1;
    }
    
    // Initialize UART
    struct termios options;
    tcgetattr(uart, &options);
    //tty.c_cflag |= CREAD | CLOCAL;
    cfsetospeed(&options, B9600);
    cfsetispeed(&options, B9600);
    
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    options.c_cflag &= ~PARENB;          // No parity bit
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~CSIZE;           // Clear data size bits
    options.c_cflag |= CS8;              // 8 data bits
    
    tcsetattr(uart, TCSANOW, &options);

    while (true) {
        //char data = '@';
        uint8_t data = 69;
        write(uart, &data, sizeof(data));
        std::cout << "sent " << static_cast<int>(data) << std::endl;
        usleep(1000000);

        // Read incoming data
        uint8_t receivedData;
        if (read(uart, &receivedData, sizeof(receivedData)) > 0) {
            std::cout << "Response: " << static_cast<int>(receivedData) << std::endl;
        }
    }

    close(uart);
    return 0;
}
