#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>

int main() {
    int serial_port;
    const char *device = "/dev/ttyAMA0"; // Default serial port on Raspberry Pi
    const int baud_rate = 115200; // ESP32 standard baud rate

    // Initialize WiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "Unable to start wiringPi" << std::endl;
        return 1;
    }

    // Open the serial port
    if ((serial_port = serialOpen(device, baud_rate)) < 0) {
        std::cerr << "Unable to open serial device" << std::endl;
        return 1;
    }

    // Main loop to read and write data
    while (true) {
        // Check if data is available to read
        if (serialDataAvail(serial_port)) {
            char data = serialGetchar(serial_port);
            std::cout << data;
            std::cout.flush();
        }

        // You can also write data to the serial port if needed
        // serialPutchar(serial_port, 'A');
    }

    // Close the serial port
    serialClose(serial_port);

    return 0;
}