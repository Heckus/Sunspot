#ifndef DATA_H
#define DATA_H

#include <mutex>
#include <iostream>

class Data {
public:
    // Constructor
    Data();

    // Getters
    double getThetaAngle();
    double getBetaAngle();
    int getBatteryLevel();
    int getMode();
    bool getButtonState(int buttonIndex);
    string getLed0Color();

    // Setters
    void setThetaAngle(double angle);
    void setBetaAngle(double angle);
    void setBatteryLevel(int level);
    void setMode(int mode);
    void setButtonState(int buttonIndex, bool state);
    void setLed0Color(string color);

    // Print function
    void printData();

private:
    double thetaAngle;
    double betaAngle;
    string led0Color;
    int batteryLevel;
    int mode;
    bool buttonStates[3]; // Assuming there are 10 buttons

    int width;
    int height;

    volatile int serialWIREFd;
    std::mutex mtx; // Mutex for thread safety
};

#endif // DATA_H