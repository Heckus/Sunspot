#include "OS_tools.h"


namespace fs = std::filesystem;

Data::Data() : thetaAngle(0), betaAngle(0), batteryLevel(0), mode(0), framerate(0), width(0), height(0), baudrate(0), serialFd(-1), batteryMonitor(1, 0x41) {
    std::fill(std::begin(buttonStates), std::end(buttonStates), false);
}

Data::~Data() {
    release();
}

void Data::setDeltatheta(int deltatheta) {
    std::lock_guard<std::mutex> lock(mtx);
    this->deltatheta = deltatheta;
}

void Data::setDeltabeta(int deltabeta) {
    std::lock_guard<std::mutex> lock(mtx);
    this->deltabeta = deltabeta;
}

void Data::setThetaAngle(int angle) {
    std::lock_guard<std::mutex> lock(mtx);
    if (angle < 90) {
        angle = 90;
    } else if (angle > -90) {
        angle = -90;
    }
    thetaAngle = angle;
}

void Data::setBetaAngle(int angle) {
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

void Data::setButtonState(int state) {
    std::lock_guard<std::mutex> lock(mtx);
    for (int i = 0; i < 3; ++i) {
        buttonStates[i] = (state & (1 << i)) != 0;
    }
}

void Data::setLed0Color(std::string color) {
    std::lock_guard<std::mutex> lock(mtx);
    led0Color = color;
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

void Data::setbaudrate(int baudrate) {
    std::lock_guard<std::mutex> lock(mtx);
    this->baudrate = baudrate;
}

int Data::getDeltatheta() {
    std::lock_guard<std::mutex> lock(mtx);
    return deltatheta;
}

int Data::getDeltabeta() {
    std::lock_guard<std::mutex> lock(mtx);
    return deltabeta;
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


int Data::getbaudrate() {
    std::lock_guard<std::mutex> lock(mtx);
    return baudrate;
}

int Data::getserialFd() {
    std::lock_guard<std::mutex> lock(mtx);
    return serialFd;
}

void Data::printData() {
    std::lock_guard<std::mutex> lock(mtx);
    std::cout << "\n══════════════════ System Status ══════════════════\n\n"
              << "╔═══════════ Angles ═══════════╗\n"
              << "║  Theta: " << std::setw(20) << thetaAngle << "° ║\n"
              << "║  Beta:  " << std::setw(20) << betaAngle  << "° ║\n"
              << "╚═══════════════════════════════╝\n\n"
              << "╔═══════════ System ═══════════╗\n"
              << "║  Battery Level: " << std::setw(13) << batteryLevel << "% ║\n"
              << "║  Mode:          " << std::setw(13) << mode << " ║\n"
              << "║  LED Color:     " << std::setw(13) << led0Color << " ║\n"
              << "╚═══════════════════════════════╝\n\n"
              << "╔═══════════ Camera ═══════════╗\n"
              << "║  Framerate: " << std::setw(16) << framerate << " ║\n"
              << "║  Width:     " << std::setw(16) << width << " ║\n"
              << "║  Height:    " << std::setw(16) << height << " ║\n"
              << "╚═══════════════════════════════╝\n\n"
              << "╔═══════════ Serial ═══════════╗\n"
              << "║  Baudrate:  " << std::setw(15) << baudrate << " ║\n"
              << "║  Serial FD: " << std::setw(15) << serialFd << " ║\n"
              << "╚═══════════════════════════════╝\n\n"
              << "╔═══════════ Tracking ═══════════╗\n"
              << "║  State: " << std::setw(19) << trackingstate << " ║\n"
              << "╚═══════════════════════════════╝\n\n"
              << "╔═══════════ Pipelines ═══════════╗\n"
              << "║  Input:  " << inputpipeline << " ║\n";
    
    for (int i = 0; i < 4; i++) {
        std::cout << "║  Output" << i << ": " << outputpipline[i] << " ║\n";
    }
    std::cout << "╚════════════════════════════════╝\n\n";

    std::cout << "╔═══════════ Video Paths ═══════════╗\n";
    for (const auto& path : videopaths) {
        std::cout << "║  • " << path << "\n";
    }
    std::cout << "╚════════════════════════════════════╝\n";
    std::cout << "\n═══════════════════════════════════════════════════\n";
}



void Data::setserialFd() {
    std::lock_guard<std::mutex> lock(mtx);
    serialFd = serialOpen(serialwiredevice, baudrate);
    if (serialFd == -1) {
        std::cerr << "Error: Unable to open serial device" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void Data::setinputpipline() {
    std::lock_guard<std::mutex> lock(mtx);
    this->inputpipeline = "libcamerasrc ! video/x-raw, format=I420,"
                       " width=" + std::to_string(width) + ","
                       " height=" + std::to_string(height) + "," 
                       " framerate=" + std::to_string(framerate) + 
                       "/1 ! videoconvert "
                       "! appsink";
}

void Data::setvideopaths() {
    std::lock_guard<std::mutex> lock(mtx);
    int usbCount = 0;

    try {
        if (!fs::exists("/media/hecke") || !fs::is_directory("/media/hecke")) {
            std::cerr << "Error: /media/hecke directory not found" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        for (const auto& entry : fs::directory_iterator("/media/hecke")) {
            if (usbCount >= 4) break;
            
            if (fs::is_directory(entry)) {
                videopaths[usbCount] = entry.path().string();
                usbCount++;
            }
        }

        if (usbCount == 0) {
            std::cerr << "Error: No USB drives found in /media" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // Fill remaining slots with "empty"
        while (usbCount < 4) {
            videopaths[usbCount++] = "empty";
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

 void Data::setoutputpipline(){
    std::lock_guard<std::mutex> lock(mtx);
    for (int i = 0; i < 4; ++i) {
        if (videopaths[i] != "empty") {
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
            std::string timestamp = oss.str();
            outputpipline[i] = "appsrc ! videoconvert ! x264enc tune=zerolatency ! mp4mux ! filesink location=" + videopaths[i] + "/" + timestamp + ".mp4";
        }
    }
 }


void Data::createwriters() {
    std::lock_guard<std::mutex> lock(mtx);
    for (int i = 0; i < 4; ++i) {
        if (videopaths[i] != "empty") {
            cv::VideoWriter writer(outputpipline[i], cv::CAP_GSTREAMER, 0, framerate, cv::Size(width, height), true);
            if (!writer.isOpened()) {
                std::cerr << "Error: Could not open the video file for writing at " << videopaths[i] << std::endl;
                std::exit(EXIT_FAILURE);
            }
            writers[i] = writer;
        }
    }
}

void Data::writeframes() {
    std::lock_guard<std::mutex> lock(mtx);
    for (int i = 0; i < 4; ++i) {
        if (videopaths[i] != "empty") {
            writers[i] << currentframe;
        }
    }
}


 void Data::createcamera(){
    std::lock_guard<std::mutex> lock(mtx);
    cv::VideoCapture camera(inputpipeline, cv::CAP_GSTREAMER);
    if (!camera.isOpened()) {
        std::cout << "Error: VideoCapture not opened" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    this->camera = camera;
}

void Data::release() {
    std::lock_guard<std::mutex> lock(mtx);
    this->camera.release();
    serialClose(serialFd);
    for (int i = 0; i < 4; ++i) {
        if (writers[i].isOpened()) {
            writers[i].release();
        }
    }
}

void Data::updateframe(){
    std::lock_guard<std::mutex> lock(mtx);
    this->camera >> this->currentframe;
    if (currentframe.empty()) {
        std::cerr << "Error: Frame is empty" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

cv::Mat Data::getframe(){
    std::lock_guard<std::mutex> lock(mtx);
    return currentframe;
}

void Data::setwiringPi(){
    std::lock_guard<std::mutex> lock(mtx);
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi!" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void Data::setTrackingState(int state) {
    std::lock_guard<std::mutex> lock(mtx);
    trackingstate = state;
}
