#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <string>
#include <signal.h>

volatile bool running = true;

void handleCtrlC(int signal) {
    running = false;
}

int main() {
    signal(SIGINT, handleCtrlC); // Handle Ctrl+C for clean exit

    const char *serialDevice = "/dev/ttyAMA0"; // Adjust if using a USB-serial adapter
    const int baudRate = 9600;

    // Initialize WiringPi and open the serial port
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi!" << std::endl;
        return 1;
    }

    int serialFd = serialOpen(serialDevice, baudRate);
    if (serialFd == -1) {
        std::cerr << "Failed to open serial device!" << std::endl;
        return 1;
    }

    std::cout << "Communication started. Type a message to send to Arduino." << std::endl;

    while (running) {
        std::string userInput;
        std::cout << "You: ";
        std::getline(std::cin, userInput);

        // Send user input to Arduino
        serialPuts(serialFd, (userInput + '\n').c_str()); // Send message with newline
        serialFlush(serialFd); // Ensure the message is sent immediately
        delay(100); // Small delay to allow Arduino to process the message

        // Wait for response
        std::string response;
        while (serialDataAvail(serialFd) > 0) {
            char c = serialGetchar(serialFd);
            if (c == '\n') break; // End of message
            response += c;
        }

        if (!response.empty()) {
            std::cout << "Arduino: " << response << std::endl;
        }
    }

    serialClose(serialFd);
    std::cout << "Exiting..." << std::endl;
    return 0;
}

// Compile with: g++ -o directpi /home/hecke/Code/GitRepositories/Sunspot/serial/src/directpi.cpp -lwiringPi
