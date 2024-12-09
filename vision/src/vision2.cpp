#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <signal.h>
#include <cstdlib>
#include <ctime>

#define width 640
#define height 480
#define framerate 30

bool running = true;

void signalHandler(int signum) {
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);

    // Old pipeline
    //std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=I420 ! videoconvert ! videobalance contrast=1.5 brightness=-0.1 saturation=1.2 ! appsink";
    //cv::VideoCapture camera(pipeline, cv::CAP_GSTREAMER);

    // New pipeline
    std::string pipeline1 = "libcamerasrc ! video/x-raw, format=I420,"
                       " width=" + std::to_string(width) + ","
                       " height=" + std::to_string(height) + "," 
                       " framerate=" + std::to_string(framerate) + "/1 ! videoconvert "
                       "! appsink";

    std::string pipeline2 = "libcamerasrc ! video/x-raw, format=NV12,"
                       " width=" + std::to_string(width) + ","
                       " height=" + std::to_string(height) + "," 
                       " framerate=" + std::to_string(framerate) + "/1 ! videoconvert "
                       " ! videobalance contrast=1.5 brightness=-0.1 saturation=1.2 ! appsink";

    //setenv("GST_DEBUG", "2", 1);  // Set GStreamer debug level to 3
    cv::VideoCapture camera(pipeline1, cv::CAP_GSTREAMER);
    
    std::srand(std::time(nullptr));
    int random_number = std::rand();
    std::string outputlocation = "/home/hecke/Code/output" + std::to_string(random_number) + ".mp4";

    if (!camera.isOpened()) {
        std::cerr << "Cannot open camera" << std::endl;
        return -1;
    }

    std::string output_pipeline = "appsrc ! videoconvert ! x264enc tune=zerolatency ! mp4mux ! filesink location=" + outputlocation;
    cv::VideoWriter writer(output_pipeline, cv::CAP_GSTREAMER, 0, framerate, cv::Size(width, height), true);
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open the video file for writing" << std::endl;
        return -1;
    }

    cv::CascadeClassifier ball_cascade;
    if (!ball_cascade.load("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")) {
        std::cerr << "Error loading face cascade classifier" << std::endl;
        return -1;
    }


    std::cout << "Press 1 for normal webcam view\n";
    std::cout << "Press 2 for ball detection\n";
    std::cout << "Press 3 for ball detection on video file\n";
    std::cout << "Press Ctrl+C to exit\n";

    char input;
    cv::Mat frame, gray;

    while (running) {
        std::cin >> input;

        if (input == '1') {
            while (running) {
                camera >> frame;
                if (frame.empty()) break;
                
                cv::rotate(frame, frame, cv::ROTATE_180);  // Rotate 180 degrees

                writer.write(frame);
                cv::imshow("Webcam", frame);
                if (cv::waitKey(1000/framerate) == 27) break;  // Delay to maintain correct framerate
            } 

            cv::destroyAllWindows();
        }
        else if (input == '2') {
            while (running) {
                camera >> frame;
                if (frame.empty()) {
                    std::cerr << "Camera disconnected or frame capture failed" << std::endl;
                    return -1;
                }
                cv::rotate(frame, frame, cv::ROTATE_180);  // Rotate 180 degrees

                
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Rect> balls;
                ball_cascade.detectMultiScale(gray, balls, 1.1, 4);

                for (const auto& ball : balls) {
                    cv::rectangle(frame, ball, cv::Scalar(0, 0, 255), 2);
                    cv::putText(frame, "ball", 
                              cv::Point(ball.x, ball.y - 5),
                              cv::FONT_HERSHEY_SIMPLEX, 0.8,
                              cv::Scalar(0, 0, 255), 2);
                }

                cv::imshow("Ball Detection", frame);
                if (cv::waitKey(1000/framerate) == 27) break;  // Delay to maintain correct framerate
            }
            cv::destroyAllWindows();
        }
        else if (input == '3') {
            std::string videoPath;
            std::cout << "Enter the path to the video file: ";
            std::cin >> videoPath;

            cv::VideoCapture cap(videoPath);
            if (!cap.isOpened()) {
                std::cerr << "Error opening video file" << std::endl;
                continue;
            }

            while (running) {
                cap >> frame;
                if (frame.empty()) break;
               

                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Rect> balls;
                ball_cascade.detectMultiScale(gray, balls, 1.1, 4);

                for (const auto& ball : balls) {
                    cv::rectangle(frame, ball, cv::Scalar(0, 0, 255), 2);
                    cv::putText(frame, "ball", 
                              cv::Point(ball.x, ball.y - 5),
                              cv::FONT_HERSHEY_SIMPLEX, 0.8,
                              cv::Scalar(0, 0, 255), 2);
                }int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
               


                cv::imshow("Ball Detection on Video", frame);
                cv::imshow("Gray Image", gray);
                if (cv::waitKey(1) == 27) break;
            }
            cap.release();
            cv::destroyAllWindows();
        }
    }

    camera.release();
    writer.release();
    return 0;
}
