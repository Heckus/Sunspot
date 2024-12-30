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