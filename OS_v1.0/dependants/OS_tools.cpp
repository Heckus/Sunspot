#include "OS_tools.h"
namespace fs = std::filesystem;

Data::Data() : thetaAngle(0), betaAngle(0), batteryLevel(0), mode(0), framerate(0), width(0), height(0), baudrate(0), serialWIREFd(-1) {
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

void Data::batteryInit() {
    std::lock_guard<std::mutex> lock(mtx);
    try {batteryMonitor = INA219(1,0x41);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
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
    this->inputpipline = "libcamerasrc ! video/x-raw, format=I420,"
                       " width=" + width + ","
                       " height=" + height + "," 
                       " framerate=" + framerate + 
                       "/1 ! videoconvert "
                       "! appsink";
}

void Data::setvideopaths() {
    std::lock_guard<std::mutex> lock(mtx);
    while (true) {
        int index = 0;
        for (auto& path : fs::directory_iterator("/media/pi")) {
            if (index < 4 && fs::is_directory(path)) {
                videopaths[index] = path.path().string();
                index++;
            }
        }
        for (; index < 4; index++) {
            videopaths[index] = "empty";
        }
        if (index > 0) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
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
    Data.camera = camera;
}

void Data::release() {
    std::lock_guard<std::mutex> lock(mtx);
    Data.camera.release();
    SerialClose(serialFd);
    for (int i = 0; i < 4; ++i) {
        if (writers[i].isOpened()) {
            writers[i].release();
        }
    }
}

void Data::updateframe(){
    std::lock_guard<std::mutex> lock(mtx);
    Data.camera >> Data.currentframe;
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

