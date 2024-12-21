#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <thread>
#include <regex>
#include <mutex>

volatile bool running = true;
volatile int serialWIREFd;
volatile int serialUSBFd;


int theta = 0;
int beta = 0;
std::string led0 = "blue";
int bat = 98;
int switchstate = 0;
int buttonstate = 0;
bool ledtoggle = 0;

void handleCtrlC(int signal) {
    running = false;
    std::string reset;
    reset = "RESET";
    serialPuts(serialWIREFd, (reset + '\n').c_str());

    serialClose(serialWIREFd);
    serialClose(serialUSBFd);
}



// New function for handling input
void workthread() {
    while (running) {
        theta += (rand() % 3 - 1); // Add random noise between -1 and 1
        beta += (rand() % 3 - 1);  // Add random noise between -1 and 1

        // Clamp values to stay within -90 and 90
        if (theta > 90) theta = 90;
        if (theta < -90) theta = -90;
        if (beta > 90) beta = 90;
        if (beta < -90) beta = -90;

        // Toggle LED color
        static int loopCounter = 0;
        loopCounter++;
        if (loopCounter % 2 == 0) {
            led0 = (led0 == "red") ? "white" : "red";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Add delay
    }
}


// Modify receiveMessages to use mutex
void communicationthread(int serialFd) {
    std::string buffer;
    const int STATUS_INTERVAL = 500;
    auto lastStatusTime = std::chrono::steady_clock::now();

    while (running) {
        while (serialDataAvail(serialFd) > 0) {  // Process all available data immediately
            char c = serialGetchar(serialFd);
            if (c == '\n') {
                std::regex pattern(R"(MC-THETA:(-?\d{1,2})BETA:(-?\d{1,2})LED0:(\w*)BAT:(\d{1,3})MODE:(\d)INPUT:(\d{3}))");
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
                    //std::cout << "\033[2J\033[H";

                    // Print updates without buffering
                    std::cout << "THETA: " << theta << "\n"
                              << "BETA: " << beta << "\n"
                              << "LED0: " << led0 << "\n"
                              << "BAT: " << bat << "\n"
                              << "MODE: " << switchstate << "\n"
                              << "INPUT: " << buttonstate << std::endl;
                } else {
                    std::cerr << "Received Raw message: " << buffer << std::endl;
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

    const char *serialWIREDevice = "/dev/ttyAMA0"; //for wired serial with pin 14 15
    const char *serialUSBDevice = "/dev/ttyACM0"; // for USB serial

    const int baudRate = 115200;
    //115200

    // Initialize WiringPi and open the serial port
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi!" << std::endl;
        return 1;
    }

    serialWIREFd = serialOpen(serialWIREDevice, baudRate);
    if (serialWIREFd == -1) {
        std::cerr << "Failed to open WIRE serial device!" << std::endl;
        return 1;
    }
    serialUSBFd = serialOpen(serialUSBDevice, baudRate);
    if (serialUSBFd == -1) {
        std::cerr << "Failed to open USB serial device!" << std::endl;
        return 1;
    }

    // Wait fo the controller to boot
    std::string bootcommand = "BOOT";
    serialPuts(serialWIREFd, (bootcommand + '\n').c_str()); // Send message with newline
    std::thread communication(communicationthread, serialWIREFd);
    std::thread work(workthread);

    // Wait for both threads to finish
    work.join();
    communication.join();

    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Add delay

    std::string reset = "RESET";
    serialPuts(serialWIREFd, (reset + '\n').c_str());
    serialClose(serialWIREFd);
    serialClose(serialUSBFd);
    std::cout << "Exiting..." << std::endl;
    return 0;
}

// Compile with: g++ -o directpi /home/hecke/Code/GitRepositories/Sunspot/serial/src/directpi.cpp -lwiringPi
