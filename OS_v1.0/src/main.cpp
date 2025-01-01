//Serial
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <thread>
#include <regex>
#include <mutex>
//Vision
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <signal.h>
#include <cstdlib>
#include <ctime>
//Battery
#include "INA219.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
//Dependants
#include "OS_tools.h"
#include "INA219.h"

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
      
   }
}

void VideoProccessingThread(Data &OsData){
    while(running){
        
    }
    
}

void SerialThread(Data &OsData){
    while(running){
        
    }
}

void BatteryThread(Data &OsData){
    while(running){
        
    }
}

void ControlThread(Data &OsData){
    while(running){
        
    }
}

int main(){
    signal(SIGINT, signalHandler);
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

    VideoCapture.join();
    Videoproccessing.join();
    Serial.join();
    Battery.join();
    Control.join();

    serialPuts(getserialFd(), (OsData.reset + '\n').c_str());
    std::cout << std::endl;
    std::cout << "Exiting..." << std::endl;
    return 0;
}