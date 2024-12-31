#include "OS_tools.h"

Data::Data() : thetaAngle(0), betaAngle(0), batteryLevel(0), mode(0), framerate(0), width(0), height(0), baudrate(0), serialWIREFd(-1) {
    std::fill(std::begin(buttonStates), std::end(buttonStates), false);
}

void Data::createcamera(std::string pipeline){
    cv::VideoCapture camera(pipeline, cv::CAP_GSTREAMER);
    if (!camera.isOpened()) {
        std::cout << "Error: VideoCapture not opened" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    Data.camera = camera;
}

void Data::createwriter(std::string pipeline){
    cv::VideoWriter writer(pipeline, cv::CAP_GSTREAMER, 0, Data.framerate, cv::Size(Data.width, Data.height), true);
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open the video file for writing" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    Data.writer = writer;
}

void Data::updateframe(){
    Data.camera >> Data.currentframe;
    if (currentframe.empty()) {
        std::cerr << "Error: Frame is empty" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}
















void Data::setThetaAngle(double angle) {
    std::lock_guard<std::mutex> lock(mtx);
    if (angle < 90) {
        angle = 90;
    } else if (angle > -90) {
        angle = -90;
    }
    thetaAngle = angle;
}

void Data::setBetaAngle(double angle) {
    std::lock_guard<std::mutex> lock(mtx);
    if (angle < 90) {
        angle = 90;
    } else if (angle > -90) {
        angle = -90;
    }
    betaAngle = angle;
}

void Data::setBatteryLevel(int level) {
    std::lock_guard<std::mutex> lock(mtx);
    batteryLevel = level;
}

void Data::setMode(int mode) {
    std::lock_guard<std::mutex> lock(mtx);
    this->mode = mode;
}

void Data::setButtonState(int buttonIndex, bool state) {
    if (buttonIndex >= 0 && buttonIndex < 3) {
        std::lock_guard<std::mutex> lock(mtx);
        buttonStates[buttonIndex] = state;
    }
}

void Data::setLed0Color(std::string color) {
    std::lock_guard<std::mutex> lock(mtx);
    led0Color = color;
}

void Data::setinputpipline(std::string pipline) {
    std::lock_guard<std::mutex> lock(mtx);
    this->inputpipline = pipline;
}

void Data::setoutputpipline(std::string pipline) {
    std::lock_guard<std::mutex> lock(mtx);
    this->outputpipline = pipline;
}

void Data::setframerate(int framerate) {
    std::lock_guard<std::mutex> lock(mtx);
    this->framerate = framerate;
}

void Data::setwidth(int width) {
    std::lock_guard<std::mutex> lock(mtx);
    this->width = width;
}

void Data::setheight(int height) {
    std::lock_guard<std::mutex> lock(mtx);
    this->height = height;
}

void Data::setvideopath(std::string path) {
    std::lock_guard<std::mutex> lock(mtx);
    this->videopath = path;
}

void Data::setbaudrate(int baudrate) {
    std::lock_guard<std::mutex> lock(mtx);
    this->baudrate = baudrate;
}

void Data::setserialWIREFd(int fd) {
    std::lock_guard<std::mutex> lock(mtx);
    this->serialWIREFd = fd;
}

int Data::getThetaAngle() {
    std::lock_guard<std::mutex> lock(mtx);
    return thetaAngle;
}

int Data::getBetaAngle() {
    std::lock_guard<std::mutex> lock(mtx);
    return betaAngle;
}

int Data::getBatteryLevel() {
    std::lock_guard<std::mutex> lock(mtx);
    return batteryLevel;
}

int Data::getMode() {
    std::lock_guard<std::mutex> lock(mtx);
    return mode;
}

bool Data::getButtonState(int buttonIndex) {
    if (buttonIndex >= 0 && buttonIndex < 3) {
        std::lock_guard<std::mutex> lock(mtx);
        return buttonStates[buttonIndex];
    }
    return false;
}

std::string Data::getLed0Color() {
    std::lock_guard<std::mutex> lock(mtx);
    return led0Color;
}

std::string Data::getinputpipline() {
    std::lock_guard<std::mutex> lock(mtx);
    return inputpipline;
}

std::string Data::getoutputpipline() {
    std::lock_guard<std::mutex> lock(mtx);
    return outputpipline;
}

int Data::getframerate() {
    std::lock_guard<std::mutex> lock(mtx);
    return framerate;
}

int Data::getwidth() {
    std::lock_guard<std::mutex> lock(mtx);
    return width;
}

int Data::getheight() {
    std::lock_guard<std::mutex> lock(mtx);
    return height;
}

std::string Data::getvideopath() {
    std::lock_guard<std::mutex> lock(mtx);
    return videopath;
}

int Data::getbaudrate() {
    std::lock_guard<std::mutex> lock(mtx);
    return baudrate;
}

int Data::getserialWIREFd() {
    std::lock_guard<std::mutex> lock(mtx);
    return serialWIREFd;
}

void Data::printData() {
    std::lock_guard<std::mutex> lock(mtx);
    std::cout << "Theta Angle: " << thetaAngle << std::endl;
    std::cout << "Beta Angle: " << betaAngle << std::endl;
    std::cout << "Battery Level: " << batteryLevel << std::endl;
    std::cout << "Mode: " << mode << std::endl;
    std::cout << "Button States: ";
    for (bool state : buttonStates) {
        std::cout << state << " ";
    }
    std::cout << std::endl;
    std::cout << "LED0 Color: " << led0Color << std::endl;
    std::cout << "Input Pipeline: " << inputpipline << std::endl;
    std::cout << "Output Pipeline: " << outputpipline << std::endl;
    std::cout << "Framerate: " << framerate << std::endl;
    std::cout << "Width: " << width << std::endl;
    std::cout << "Height: " << height << std::endl;
    std::cout << "Video Path: " << videopath << std::endl;
    std::cout << "Baudrate: " << baudrate << std::endl;
    std::cout << "Serial Wire FD: " << serialWIREFd << std::endl;
}