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
std::mutex serialMutex;

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
void handleInput(int serialFd) {
    std::string userInput;
    std::cout << "Communication started. Type a message to send to Controller." << std::endl;
    while (running) {
        if (std::cin >> userInput) {
            std::string message;
            
            if (userInput == "a") {
                message = "THETA:10";
            } else if (userInput == "d") {
                message = "THETA:-10";
            } else if (userInput == "w") {
                message = "BETA:10";
            } else if (userInput == "s") {
                message = "BETA:-10";
            } else if(userInput == "l1") {
                message = "LED0:red";
            }else if(userInput == "l2"){
                message = "LED0:green";
            } else if (userInput == "q") {
                running = false;
                break;
            }
            
            if (!message.empty()) {
                std::lock_guard<std::mutex> lock(serialMutex);
                serialPuts(serialFd, (message + '\n').c_str());
                fflush(stdout);
            }
        }
    }
}

// Modify receiveMessages to use mutex
void receiveMessages(int serialFd) {
    std::string buffer;
    std::string status = "STATUS";
    const int STATUS_INTERVAL = 100;
    auto lastStatusTime = std::chrono::steady_clock::now();

    while (running) {
        while (serialDataAvail(serialFd) > 0) {  // Process all available data immediately
            char c = serialGetchar(serialFd);
            if (c == '\n') {
                std::regex pattern(R"(THETA:(-?\d+)\s+BETA:(-?\d+)\s+LED0:(\w+)\s+BAT:(\d+)\s+SWITCHSTATE:(\d+)\s+BUTTONSTATE:(\d+))");
                std::smatch matches;
                if (std::regex_search(buffer, matches, pattern)) {
                    theta = std::stoi(matches[1]);
                    beta = std::stoi(matches[2]);
                    led0 = matches[3];
                    bat = std::stoi(matches[4]);
                    switchstate = std::stoi(matches[5]);
                    buttonstate = std::stoi(matches[6]);
                    
                    // Clear terminal (Linux/Unix)
                    std::cout << "\033[2J\033[H";
                    // Print updates without buffering
                    std::cout << "THETA: " << theta << "\n"
                              << "BETA: " << beta << "\n"
                              << "LED0: " << led0 << "\n"
                              << "BAT: " << bat << "\n"
                              << "SWITCHSTATE: " << switchstate << "\n"
                              << "BUTTONSTATE: " << buttonstate << std::endl;
                            
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
            std::lock_guard<std::mutex> lock(serialMutex);
            serialPuts(serialFd, (status + '\n').c_str());
            fflush(stdout);
            lastStatusTime = now;
        }
    }
}





   

int main() {
    signal(SIGINT, handleCtrlC); // Handle Ctrl+C for clean exit

    const char *serialDevice = "/dev/ttyAMA0"; // Adjust if using a USB-serial adapter
    //const char *serialDevice = "/dev/ttyACM0"; 
    const int baudRate = 19200;
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

    std::thread receiverThread(receiveMessages, serialFd);
    std::thread inputThread(handleInput, serialFd);

    // Wait for both threads to finish
    inputThread.join();
    receiverThread.join();

    delay(500); // Wait for the last message to be sent

    std::string reset = "RESET";
    serialPuts(serialFd, (reset + '\n').c_str());
    serialClose(serialFd);
    std::cout << "Exiting..." << std::endl;
    return 0;
}

// Compile with: g++ -o directpi /home/hecke/Code/GitRepositories/Sunspot/serial/src/directpi.cpp -lwiringPi
