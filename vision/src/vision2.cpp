#include <opencv2/opencv.hpp>
#include <iostream>
#include <signal.h>

bool running = true;

void signalHandler(int signum) {
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);

    // Old pipeline
    std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=I420 !  videoconvert ! videobalance contrast=1.5 brightness=-0.1 saturation=1.2 ! appsink";
    cv::VideoCapture camera(pipeline, cv::CAP_GSTREAMER);

    std::cout << "Camera opened successfully" << std::endl;
    if (!camera.isOpened()) {
        std::cerr << "Cannot open camera" << std::endl;
        return -1;
    }

    cv::CascadeClassifier ball_cascade;
    if (!ball_cascade.load("/home/hecke/Code/GitRepositories/Sunspot/vision/models/cascade4.xml")) {
        std::cerr << "Error loading ball cascade classifier" << std::endl;
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

                cv::imshow("Webcam", frame);
                if (cv::waitKey(1) == 27) break;
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
                cv::imshow("Gray Image", gray);
                if (cv::waitKey(1) == 27) break;
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
                }

                cv::imshow("Ball Detection on Video", frame);
                cv::imshow("Gray Image", gray);
                if (cv::waitKey(1) == 27) break;
            }
            cap.release();
            cv::destroyAllWindows();
        }
    }

    camera.release();
    return 0;
}
