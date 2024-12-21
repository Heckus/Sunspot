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

std::thread work;
std::thread communication;


int theta = 0;
int beta = 0;
int thetachange = 0;
int betachange = 0;

std::string led0 = "blue";
int bat = 98;
int switchstate = 0;
int buttonstate = 0;
bool ledtoggle = 0;

void handleCtrlC(int signal) {
    running = false;
}



// New function for handling input
void workthread() {
    while (running) {
        thetachange = (rand() % 5 - 2); // Generate random number between -2 and 2
        betachange = (rand() % 5 - 2);  // Generate random number between -2 and 2

        
        // Toggle LED color
        static int loopCounter = 0;
        loopCounter++;
        if (loopCounter % 2 == 0) {
            led0 = (led0 == "red") ? "white" : "red";
        }

        // Decrease battery level randomly
        static int batteryCounter = 0;
        batteryCounter++;
        if (batteryCounter % 10 == 0) { // Change battery level every second (10 * 100ms)
            bat -= (rand() % 3 + 1); // Decrease battery by 1 to 3
            if (bat <= 0) {
            bat = 100; // Reset battery to 100 when it reaches 0
            }
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
        while (serialDataAvail(serialFd) > 0 && running) {  // Process all available data immediately
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
                              << "INPUT: " << matches[6].str() << std::endl; // Use matches[6].str() to preserve leading zeros
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
            std::string message = "PI-THETA:" + std::to_string(thetachange) +
                                  "BETA:" + std::to_string(betachange) +
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
    

    std::string bootcommand = "BOOT";
    serialPuts(serialWIREFd, (bootcommand + '\n').c_str()); // Send message with newline
    
    // Wait fo the controller to boot
    communication = std::thread(communicationthread, serialWIREFd);
    work = std::thread(workthread);

    // Wait for both threads to finish
    if (work.joinable()) {
        work.join();
    }
    if (communication.joinable()) {
        communication.join();
    }

    std::string reset = "RESET";
    serialPuts(serialWIREFd, (reset + '\n').c_str());
    serialClose(serialWIREFd);
    std::cout << std::endl;
    std::cout << "Exiting..." << std::endl;
    return 0;
}
