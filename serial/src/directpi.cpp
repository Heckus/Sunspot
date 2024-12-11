#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <thread>

volatile bool running = true;

void handleCtrlC(int signal) {
    running = false;
}

#include <regex>

void receiveMessages(int serialFd) {
    std::string buffer;
    while (running) {
        if (serialDataAvail(serialFd) > 0) {
            char c = serialGetchar(serialFd);
            if (c == '\n') {
                std::regex pattern(R"(THETA:(\d+)\s+BETA:(\d+)\s+LED0:(\w+)\s+BAT:(\d+)\s+STATE:(\d+))");
                std::smatch matches;
                if (std::regex_search(buffer, matches, pattern)) {
                    std::cout << "THETA: " << matches[1] << std::endl;
                    std::cout << "BETA: " << matches[2] << std::endl;
                    std::cout << "LED0: " << matches[3] << std::endl;
                    std::cout << "BAT: " << matches[4] << std::endl;
                    std::cout << "STATE: " << matches[5] << std::endl;
                } else {
                    std::cerr << "Received malformed message: " << buffer << std::endl;
                }
                buffer.clear();
            } else {
                buffer += c;
            }
        }
    }
}

int main() {
    signal(SIGINT, handleCtrlC); // Handle Ctrl+C for clean exit

    const char *serialDevice = "/dev/ttyAMA0"; // Adjust if using a USB-serial adapter
    //const char *serialDevice = "/dev/ttyACM0"; 
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

    std::thread receiverThread(receiveMessages, serialFd);

    std::cout << "Communication started. Type a message to send to Controller." << std::endl;
    std::string bootcommand = "BOOT";
    serialPuts(serialFd, (bootcommand + '\n').c_str()); // Send message with newline
        
  

    while (running) {
        char userInput = getchar();
        std::string message;
        
        if (userInput == 'a') {
            message = "THETA:5";
        } else if (userInput == 'd') {
            message = "THETA:-5";
        } else if (userInput == 'w') {
            message = "BETA:5";
        } else if (userInput == 's') {
            message = "BETA:-5";
        } else if (userInput == 3) { // Ctrl+C
            running = false;
            break;
        } else {
            message = "STATUS";
        }
        
        serialPuts(serialFd, (message + '\n').c_str());
        fflush(stdout);
    }


    receiverThread.join();

    serialClose(serialFd);
    std::cout << "Exiting..." << std::endl;
    return 0;
}

// Compile with: g++ -o directpi /home/hecke/Code/GitRepositories/Sunspot/serial/src/directpi.cpp -lwiringPi
