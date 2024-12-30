INA219 use example
```cpp
#include "INA219.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main() {
    try {
        INA219 ina219(1, 0x41);  // Use I2C bus 1, address 0x41

        while (true) {
            float bus_voltage = ina219.getBusVoltage_V();
            float current = ina219.getCurrent_mA();
            float power = ina219.getPower_W();
            float percentage = ina219.getBatteryPercentage();

            std::cout << "Load Voltage: " << std::fixed << std::setprecision(3) 
                      << bus_voltage << " V" << std::endl;
            std::cout << "Current: " << std::fixed << std::setprecision(6) 
                      << current/1000.0f << " A" << std::endl;
            std::cout << "Power: " << std::fixed << std::setprecision(3) 
                      << power << " W" << std::endl;
            std::cout << "Battery: " << std::fixed << std::setprecision(1) 
                      << percentage << "%" << std::endl;
            std::cout << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
```



Vision code from vision2

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <signal.h>
#include <cstdlib>
#include <ctime>

#define width 640
#define height 480
#define framerate 30

bool running = true;

void signalHandler(int signum) {
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);

    // Old pipeline
    //std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=I420 ! videoconvert ! videobalance contrast=1.5 brightness=-0.1 saturation=1.2 ! appsink";
    //cv::VideoCapture camera(pipeline, cv::CAP_GSTREAMER);

    // New pipeline
    std::string pipeline1 = "libcamerasrc ! video/x-raw, format=I420,"
                       " width=" + std::to_string(width) + ","
                       " height=" + std::to_string(height) + "," 
                       " framerate=" + std::to_string(framerate) + "/1 ! videoconvert "
                       "! appsink";

    std::string pipeline2 = "libcamerasrc ! video/x-raw, format=NV12,"
                       " width=" + std::to_string(width) + ","
                       " height=" + std::to_string(height) + "," 
                       " framerate=" + std::to_string(framerate) + "/1 ! videoconvert "
                       " ! videobalance contrast=1.5 brightness=-0.1 saturation=1.2 ! appsink";

    //setenv("GST_DEBUG", "2", 1);  // Set GStreamer debug level to 3
    cv::VideoCapture camera(pipeline1, cv::CAP_GSTREAMER);
    

    std::srand(std::time(nullptr));
    int random_number = std::rand();
    std::string outputlocation = "/home/hecke/Code/output" + std::to_string(random_number) + ".mp4";

    if (!camera.isOpened()) {
        std::cerr << "Cannot open camera" << std::endl;
        return -1;
    }

    std::string output_pipeline = "appsrc ! videoconvert ! x264enc tune=zerolatency ! mp4mux ! filesink location=" + outputlocation;
    cv::VideoWriter writer(output_pipeline, cv::CAP_GSTREAMER, 0, framerate, cv::Size(width, height), true);
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open the video file for writing" << std::endl;
        return -1;
    }

    cv::CascadeClassifier ball_cascade;
    if (!ball_cascade.load("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")) {
        std::cerr << "Error loading face cascade classifier" << std::endl;
        return -1;
    }


    std::cout << "Press 1 for normal webcam view\n";
    std::cout << "Press 2 for ball detection\n";
    std::cout << "Press 3 for ball detection on video file\n";
    std::cout << "Press Ctrl+C to exit\n";

    char input;
    cv::Mat frame, gray;

    while (running) {
        std::cin >> input;

        if (input == '1') {
            while (running) {
                camera >> frame;
                if (frame.empty()) break;
                
                cv::rotate(frame, frame, cv::ROTATE_180);  // Rotate 180 degrees

                writer.write(frame);
                cv::imshow("Webcam", frame);
                if (cv::waitKey(1000/framerate) == 27) break;  // Delay to maintain correct framerate
            } 

            cv::destroyAllWindows();
        }
        else if (input == '2') {
            while (running) {
                camera >> frame;
                if (frame.empty()) {
                    std::cerr << "Camera disconnected or frame capture failed" << std::endl;
                    return -1;
                }
                cv::rotate(frame, frame, cv::ROTATE_180);  // Rotate 180 degrees

                
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Rect> balls;
                ball_cascade.detectMultiScale(gray, balls, 1.1, 4);

                for (const auto& ball : balls) {
                    cv::rectangle(frame, ball, cv::Scalar(0, 0, 255), 2);
                    cv::putText(frame, "ball", 
                              cv::Point(ball.x, ball.y - 5),
                              cv::FONT_HERSHEY_SIMPLEX, 0.8,
                              cv::Scalar(0, 0, 255), 2);
                }

                cv::imshow("Ball Detection", frame);
                if (cv::waitKey(1000/framerate) == 27) break;  // Delay to maintain correct framerate
            }
            cv::destroyAllWindows();
        }
        else if (input == '3') {
            std::string videoPath;
            std::cout << "Enter the path to the video file: ";
            std::cin >> videoPath;

            cv::VideoCapture cap(videoPath);
            if (!cap.isOpened()) {
                std::cerr << "Error opening video file" << std::endl;
                continue;
            }

            while (running) {
                cap >> frame;
                if (frame.empty()) break;
               

                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Rect> balls;
                ball_cascade.detectMultiScale(gray, balls, 1.1, 4);

                for (const auto& ball : balls) {
                    cv::rectangle(frame, ball, cv::Scalar(0, 0, 255), 2);
                    cv::putText(frame, "ball", 
                              cv::Point(ball.x, ball.y - 5),
                              cv::FONT_HERSHEY_SIMPLEX, 0.8,
                              cv::Scalar(0, 0, 255), 2);
                }int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
               


                cv::imshow("Ball Detection on Video", frame);
                cv::imshow("Gray Image", gray);
                if (cv::waitKey(1) == 27) break;
            }
            cap.release();
            cv::destroyAllWindows();
        }
    }

    camera.release();
    writer.release();
    return 0;
}

```

serial code from mcserial

```cpp
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

```