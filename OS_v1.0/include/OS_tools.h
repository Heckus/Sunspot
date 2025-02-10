#ifndef OS_TOOLS_H
#define OS_TOOLS_H
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
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
//USB
#include <filesystem>
//Dependants
#include "INA219.h"
#include <gtkmm.h>
#include <gtk/gtk.h>



/*
/usr/local/include/wiringPi
/usr/include/gtkmm-4.0/
/usr/include/gtk-4.0/**
/usr/include/**
/usr/lib/**
/usr/include/glibmm-2.4/**
/usr/include/glib-2.0
/usr/lib/x86_64-linux-gnu/glib-2.0/include
*/

class Data {
public:
    // Constructor
    Data();
    // Destructor
    ~Data();

    // Getters
    int getThetaAngle();
    int getBetaAngle();
    int getDeltatheta();
    int getDeltabeta();

    int getBatteryLevel();
    int getMode();
    bool getButtonState(int buttonIndex);
    std::string getLed0Color();


    int getframerate();
    int getwidth();
    int getheight();

    int getbaudrate();
    int getserialFd();

    // Setters
    void setThetaAngle(int angle);
    void setBetaAngle(int angle);
    void setDeltatheta(int angle);
    void setDeltabeta(int angle);
    void setBatteryLevel(int level);
    void setMode(int mode);
    void setButtonState(int state);
    void setLed0Color(std::string color);

    void setTrackingState(int state);

    void setframerate(int framerate);
    void setwidth(int width);
    void setheight(int height);
    void setinputpipline();
    void createcamera();
    void updateframe();
    cv::Mat getframe();


    void setbaudrate(int baudrate);
    void setwiringPi();
    void setserialFd();

    void setoutputpipline();
    void setvideopaths();
    void createwriters();
    void writeframes();

    void release();
 
    // Print function
    void printData();

    std::string boot = "BOOT";
    std::string reset = "RESET";

    INA219 batteryMonitor;
    
private:
    int thetaAngle = 0;
    int betaAngle = 0;
    int deltatheta = 0;
    int deltabeta = 0;

    std::string led0Color;
    int batteryLevel;
    int mode;
    bool buttonStates[3]; // Assuming there are 10 buttons

    std::string inputpipeline;
    int width;
    int height;
    int framerate;
    cv::VideoCapture camera;
    cv::Mat currentframe;

    std::string videopaths[4];
    std::string outputpipline[4];
    cv::VideoWriter writers[4];

    int baudrate;
    const char* serialwiredevice = "/dev/ttyAMA0";
    volatile int serialFd;

    int trackingstate;

    std::mutex mtx; // Mutex for thread safety
};

#endif // DATA_H