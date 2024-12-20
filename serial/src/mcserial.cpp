#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <thread>
#include <regex>
#include <mutex>

volatile bool running = true;
volatile int serialFd;


int theta = 0;
int beta = 0;
std::string led0 = "";
int bat = 0;
int switchstate = 0;
int buttonstate = 0;

bool ledtoggle = 0;

void handleCtrlC(int signal) {
    running = false;
    std::string reset;
    reset = "RESET";
    serialPuts(serialFd, (reset + '\n').c_str());
}



// New function for handling input
void workthread(int serialFd) {
    while (running) {
       
            theta += (rand() % 3 - 1); // Add random noise between -1 and 1
            beta += (rand() % 3 - 1);  // Add random noise between -1 and 1

            // Clamp values to stay within -90 and 90
            if (theta > 90) theta = 90;
            if (theta < -90) theta = -90;
            if (beta > 90) beta = 90;
            if (beta < -90) beta = -90;
        
        }
    }


// Modify receiveMessages to use mutex
void communicationthread(int serialFd) {
    std::string buffer;
    const int STATUS_INTERVAL = 100;
    auto lastStatusTime = std::chrono::steady_clock::now();

    while (running) {
        while (serialDataAvail(serialFd) > 0) {  // Process all available data immediately
            char c = serialGetchar(serialFd);
            if (c == '\n') {
                std::regex pattern(R"(MC-THETA:(-?\d{1,2})BETA:(-?\d{1,2})LED0:(\w+)BAT:(\d{1,3})MODE:(\d)INPUT:(\d{3}))");
                std::smatch matches;
                if (std::regex_search(buffer, matches, pattern)) {
                    {
                        theta = std::stoi(matches[1]);
                        beta = std::stoi(matches[2]);
                        led0 = matches[3];
                        bat = std::stoi(matches[4]);
                        switchstate = std::stoi(matches[5]);
                        buttonstate = std::stoi(matches[6]);
                    }

                    // Clear terminal (Linux/Unix)
                    std::cout << "\033[2J\033[H";
                    // Print updates without buffering
                    std::cout << "THETA: " << theta << "\n"
                              << "BETA: " << beta << "\n"
                              << "LED0: " << led0 << "\n"
                              << "BAT: " << bat << "\n"
                              << "MODE: " << switchstate << "\n"
                              << "INPUT: " << buttonstate << std::endl;
                } else {
                    std::cerr << "Received malformed message: " << buffer << std::endl;
                }
                buffer.clear();
            } else {
                buffer += c;
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStatusTime).count() >= STATUS_INTERVAL) {
            std::string message = "PI-THETA:" + std::to_string(theta) +
                                  "BETA:" + std::to_string(beta) +
                                  "LED0:" + led0 +
                                  "BAT:" + std::to_string(bat);
            serialPuts(serialFd, (message + '\n').c_str());
            fflush(stdout);
            lastStatusTime = now;
        }
    }
}





   

int main() {
    signal(SIGINT, handleCtrlC); // Handle Ctrl+C for clean exit

    const char *serialDevice = "/dev/serial0"; // Adjust if using a USB-serial adapter
    //const char *serialDevice = "/dev/ttyACM0"; 
    const int baudRate = 115200;
    //115200
    // Initialize WiringPi and open the serial port
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi!" << std::endl;
        return 1;
    }

    serialFd = serialOpen(serialDevice, baudRate);
    if (serialFd == -1) {
        std::cerr << "Failed to open serial device!" << std::endl;
        return 1;
    }

    std::string bootcommand = "BOOT";
    serialPuts(serialFd, (bootcommand + '\n').c_str()); // Send message with newline
    delay(1000); // Wait for the controller to boot
    std::thread communication(communicationthread, serialFd);
    std::thread work(workthread, serialFd);

    // Wait for both threads to finish
    work.join();
    communication.join();

    delay(500); // Wait for the last message to be sent

    std::string reset = "RESET";
    serialPuts(serialFd, (reset + '\n').c_str());
    serialClose(serialFd);
    std::cout << "Exiting..." << std::endl;
    return 0;
}

// Compile with: g++ -o directpi /home/hecke/Code/GitRepositories/Sunspot/serial/src/directpi.cpp -lwiringPi
