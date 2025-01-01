#include "../include/OS_tools.h"

volatile bool running = true;
void signalHandler(int signum) {
    running = false;
}

Data OsData = new Data();

OsData.setwidth(640);
OsData.setheight(480);                       
OsData.setframerate(30);
OsData.setbaudrate(115200);

std::thread VideoCapture;
std::thread Videoproccessing;
std::thread Serial;
std::thread Battery;
std::thread Control;

void VideoCaptureThread(Data &OsData){
   while(running){
    OsData.updateframe();
    OsData.writeframes();
    cv::imshow("CurrentFrame", OsData.currentframe);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / OsData.getframerate()));
   }
}

void VideoProccessingThread(Data &OsData){
    while(running){
        frame = OSData.getframe();
        if(OsData.getMode() == 1){
            //do nothing
        }
        else if(OsData.getMode() == 2){
            //do something
        }
        else if(OsData.getMode() == 3){
            //do something else
        }         
    }
}


void SerialThread(Data &OsData){
    while(running){
    std::string buffer;
    const int STATUS_INTERVAL = 500;
    auto lastStatusTime = std::chrono::steady_clock::now();

    while (running) {
        while (serialDataAvail(OsData.getserialFd()) > 0 && running) {  // Process all available data immediately
            char c = serialGetchar(OsData.getserialFd());
            if (c == '\n') {
                std::regex pattern(R"(MC-THETA:(-?\d{1,2})BETA:(-?\d{1,2})LED0:(\w*)BAT:(\d{1,3})MODE:(\d)INPUT:(\d{3}))");
                std::smatch matches;
                if (std::regex_search(buffer, matches, pattern)) {
                    {
                        OsData.setThetaAngle(std::stoi(matches[1]));
                        OsData.setBetaAngle(std::stoi(matches[2]));
                        //OsData.setLed0Color(matches[3]);
                        //OsData.setBatteryLevel(std::stoi(matches[4]));
                        OsData.setMode(std::stoi(matches[5]));
                        OsData.setButtonState(std::stoi(matches[6]));
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
            std::string message = "PI-THETA:" + std::to_string(OsData.getDeltatheta()) +
                                  "BETA:" + std::to_string(OsData.getDeltabeta()) +
                                  "LED0:" + OsData.getLED0Color() +
                                  "BAT:" + std::to_string(OsData.getBatteryLevel()) +;
            serialPuts(OsData.getserialFd(), (message + '\n').c_str());
            fflush(stdout);
            lastStatusTime = now;
        }
    }
    }
}

void BatteryThread(Data &OsData){
    while(running){
        OsData.batteryLevel = int(OsData.batteryMonitor.getBatteryPercentage())+;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void ControlThread(Data &OsData){
    while(running){
        
    }
}

int main(){
    signal(SIGINT, signalHandler);
    OsData.batteryInit();
    OsData.setwiringPi();
    OsData.setserialFd();
    OsData.setinputpipline();
    OsData.setvideopaths();
    OsData.setoutputpipline();
    OsData.createwriters();
    OsData.createcamera();

    VideoCapture = std::thread(VideoCaptureThread, std::ref(OsData));
    Videoproccessing = std::thread(VideoProccessingThread, std::ref(OsData));
    Serial = std::thread(SerialThread, std::ref(OsData));
    Battery = std::thread(BatteryThread, std::ref(OsData));
    Control = std::thread(ControlThread, std::ref(OsData));

    /*
    need to add i/o control(ie use mode switche to determine what to do)
    and buttons to start and stop/change modes
    */

    VideoCapture.join();
    Videoproccessing.join();
    Serial.join();
    Battery.join();
    Control.join();

    serialPuts(OsData.getserialFd(), (OsData.reset + '\n').c_str());
    std::cout << std::endl;
    std::cout << "Exiting..." << std::endl;
    return 0;
}