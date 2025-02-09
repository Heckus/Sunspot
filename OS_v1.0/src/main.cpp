#include "OS_tools.h"





volatile bool running = true;
void signalHandler(int signum) {
    running = false;

}

Data OsData;

std::thread VideoCapture;
std::thread Videoproccessing;
std::thread Serial;
std::thread Battery;
std::thread USB;
std::thread Monitoring;

void VideoCaptureThread(Data &OsData){
    while(running){
        if (!OsData.updateframe()) {  // Add error checking for frame capture
            std::cerr << "Failed to capture frame" << std::endl;
            continue;
        }
        
        OsData.writeframes();
        
        // Check if frame is empty before displaying
        if (!OsData.getframe().empty()) {
            cv::imshow("CurrentFrame", OsData.getframe());
            cv::waitKey(1000 / OsData.getframerate());  // Wait time in milliseconds based on framerate
        }
    }
    
    // Cleanup
    cv::destroyWindow("CurrentFrame");
}

void VideoProccessingThread(Data &OsData){
    cv::CascadeClassifier face_cascade;
    if (!face_cascade.load("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")) {
        std::cerr << "Error loading face cascade" << std::endl;
        return;
    }
    while(running){
        cv::Mat frame = OsData.getframe();

        std::vector<cv::Rect> faces;
        face_cascade.detectMultiScale(frame, faces);

        if (!faces.empty()) {
            OsData.setTrackingState(1);
            cv::Rect face = faces[0]; // Assuming the first detected face is the one we want
            cv::Point frame_center(frame.cols / 2, frame.rows / 2);
            cv::Point face_center(face.x + face.width / 2, face.y + face.height / 2);

            double x_diff = face_center.x - frame_center.x;
            double y_diff = face_center.y - frame_center.y;

            double x_angle = (x_diff / frame.cols) * 160.0; // 160 degrees field of view
            double y_angle = (y_diff / frame.rows) * 160.0; // 160 degrees field of view

            OsData.setDeltatheta(x_angle);
            OsData.setDeltabeta(y_angle);
        }
        else {
            OsData.setTrackingState(0);
        }
    }
}


void SerialThread(Data &OsData){
    while(running){
    std::string buffer;
    const int STATUS_INTERVAL = 500;
    auto lastStatusTime = std::chrono::steady_clock::now();
    serialPuts(OsData.getserialFd(), (OsData.boot + '\n').c_str());
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
                        //OsData.setLed0Color(matches[3]); //the mc doesnt update the color of the led0
                        //OsData.setBatteryLevel(std::stoi(matches[4])); //the mc doesnt update the battery level
                        OsData.setMode(std::stoi(matches[5]));
                        OsData.setButtonState(std::stoi(matches[6]));
                    }
                    
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
                                  "LED0:" + OsData.getLed0Color() +
                                  "BAT:" + std::to_string(OsData.getBatteryLevel());
            serialPuts(OsData.getserialFd(), (message + '\n').c_str());
            fflush(stdout);
            lastStatusTime = now;
        }
    }
    }
}

void BatteryThread(Data &OsData){
    while(running){
        OsData.setBatteryLevel(int(OsData.batteryMonitor.getBatteryPercentage()));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}


void MonitoringThread(Data &OsData){
    while(running){
        OsData.printData();
        system("clear");
        std::this_thread::sleep_for(std::chrono::seconds(100));

    }
}

void USBThread(Data &OsData){
    while(running){
        
    }
}

int main(int argc, char *argv[]){
    signal(SIGINT, signalHandler);
    if (argc != 4) {
            std::cerr << "Usage: " << argv[0] << " <width> <height> <framerate>" << std::endl;
            return 1;
        }

    int width = std::stoi(argv[1]);
    int height = std::stoi(argv[2]);
    int framerate = std::stoi(argv[3]);

    if (width <= 0 || height <= 0 || framerate <= 0) {
        std::cerr << "Invalid input: width, height, and framerate must be positive integers." << std::endl;
        return 1;
    }
    
    OsData.setwidth(width);
    OsData.setheight(height);                       
    OsData.setframerate(framerate);
    OsData.setbaudrate(115200);
   
    std::cout << "Setting up WiringPi..." << std::endl;
    OsData.setwiringPi();
    
    std::cout << "Setting up Serial connection..." << std::endl;
    OsData.setserialFd();
    
    std::cout << "Setting up input pipeline..." << std::endl;
    OsData.setinputpipline();
    
    std::cout << "Setting video paths..." << std::endl;
    OsData.setvideopaths();
    
    std::cout << "Setting up output pipeline..." << std::endl;
    OsData.setoutputpipline();

    std::cout << "Creating video writers..." << std::endl;
    OsData.createwriters();
    
    std::cout << "Setting up camera..." << std::endl;
    OsData.createcamera();

    VideoCapture = std::thread(VideoCaptureThread, std::ref(OsData));
    Videoproccessing = std::thread(VideoProccessingThread, std::ref(OsData));
    Serial = std::thread(SerialThread, std::ref(OsData));
    Battery = std::thread(BatteryThread, std::ref(OsData));
    USB = std::thread(USBThread, std::ref(OsData));
    Monitoring = std::thread(MonitoringThread, std::ref(OsData));

    /*
    need to add i/o control(ie use mode switche to determine what to do)
    and buttons to start and stop/change modes
    */

    VideoCapture.join();
    Videoproccessing.join();
    Serial.join();
    Battery.join();
    USB.join();
    Monitoring.join();

    OsData.release();
    serialPuts(OsData.getserialFd(), (OsData.reset + '\n').c_str());
    std::cout << std::endl;
    std::cout << "Exiting..." << std::endl;
    return 0;
}