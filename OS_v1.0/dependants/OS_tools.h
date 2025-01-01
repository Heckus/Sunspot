#ifndef DATA_H
#define DATA_H

#include <mutex>
#include <iostream>
#include <string>

#include <filesystem>
#include <thread>
#include <chrono>

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
    void setLed0Color(string color);

    void setframerate(int framerate);
    void setwidth(int width);
    void setheight(int height);
    void setinputpipline();
    void createcamera();
    void updateframe();
    void getframe();


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
private:
    int thetaAngle;
    int betaAngle;
    int deltatheta;
    int deltabeta;

    std::string led0Color;
    int batteryLevel;
    int mode;
    bool buttonStates[3]; // Assuming there are 10 buttons

    std::string inputpipline;
    int width;
    int height;
    int framerate;
    cv::VideoCapture camera;
    cv::Mat currentframe;

    std::string videopaths[4];
    std::string outputpipline[4];
    cv::VideoWriter writer[4];

    int baudrate;
    std::string serialwiredevice = "/dev/ttyAMA0";
   

    volatile int serialFd;
    std::mutex mtx; // Mutex for thread safety
};

#endif // DATA_H