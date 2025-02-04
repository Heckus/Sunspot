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
    OsData.updateframe();
    OsData.writeframes();
    //cv::imshow("CurrentFrame", OsData.getframe());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / OsData.getframerate()));
   }
}

void VideoProccessingThread(Data &OsData){
    while(running){
        cv::Mat frame = OsData.getframe();
        if(OsData.getMode() == 1){
            //do nothing
        }
        else if(OsData.getMode() == 2){
            //do something
        }
        else if(OsData.getMode() == 3){
            //do something else
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
        GtkWidget *window;
        GtkWidget *grid;
        GtkWidget *image;
        GtkWidget *dataLabel;

        // GTK init must be in this thread
        gtk_init(NULL, NULL);

        // Create main window
        window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_title(GTK_WINDOW(window), "Camera Monitor");
        gtk_window_set_default_size(GTK_WINDOW(window), OsData.getwidth(), OsData.getheight());
        g_signal_connect(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

        // Create grid layout
        grid = gtk_grid_new();
        gtk_container_add(GTK_CONTAINER(window), grid);

        // Image widget for camera feed
        image = gtk_image_new();
        gtk_grid_attach(GTK_GRID(grid), image, 0, 0, 1, 1);

        // Label for data display
        dataLabel = gtk_label_new("");
        gtk_grid_attach(GTK_GRID(grid), dataLabel, 1, 0, 1, 1);

        gtk_widget_show(window);

        while(running) {
            // Convert OpenCV mat to GdkPixbuf
            cv::Mat frame = OsData.getframe();
            cv::Mat rgb;
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
            GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data(
                rgb.data,
                GDK_COLORSPACE_RGB,
                false,
                8,
                rgb.cols,
                rgb.rows,
                rgb.step,
                nullptr,
                nullptr
            );
            
            // Update widgets
            gtk_image_set_from_pixbuf(GTK_IMAGE(image), pixbuf);
            g_object_unref(pixbuf);

            std::string data = 
                "Battery: " + std::to_string(OsData.getBatteryLevel()) + "%\n" +
                "Mode: " + std::to_string(OsData.getMode()) + "\n" + 
                "Theta: " + std::to_string(OsData.getThetaAngle()) + "\n" +
                "Beta: " + std::to_string(OsData.getBetaAngle());
                
            gtk_label_set_text(GTK_LABEL(dataLabel), data.c_str());
            
            while (g_main_context_pending(NULL))
                g_main_context_iteration(NULL, TRUE);
                
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            OsData.printData();
        }

        gtk_widget_destroy(window);
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

    serialPuts(OsData.getserialFd(), (OsData.reset + '\n').c_str());
    std::cout << std::endl;
    std::cout << "Exiting..." << std::endl;
    return 0;
}