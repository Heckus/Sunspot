#ifndef DATA_H
#define DATA_H

#include <mutex>
#include <iostream>
#include <string>

class Data {
public:
    // Constructor
    Data();

    // Setters
    void setThetaAngle(double angle);
    void setBetaAngle(double angle);
    void setBatteryLevel(int level);
    void setMode(int mode);
    void setButtonState(int buttonIndex, bool state);
    void setLed0Color(string color);

    void setinputpipline(std::string pipline);
    void setoutputpipline(std::string pipline);
    void setframerate(int framerate);
    void setwidth(int width);
    void setheight(int height);
    void setvideopath(std::string path);
    void setbaudrate(int baudrate);
    void setserialWIREFd(int fd);

    // Print function
    void printData();

private:
    double thetaAngle;
    double betaAngle;
    std::string led0Color;
    int batteryLevel;
    int mode;
    bool buttonStates[3]; // Assuming there are 10 buttons

    std::string inputpipline;
    std::string outputpipline;
    int width;
    int height;
    int framerate;
    std::string videopath;

    int baudrate;
    std::string serialwiredevice = "/dev/ttyAMA0";
    std::string boot = "BOOT";
    std::string reset = "RESET";

    volatile int serialWIREFd;
    std::mutex mtx; // Mutex for thread safety
};

#endif // DATA_H