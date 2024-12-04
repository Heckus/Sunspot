#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=BGR ! videoconvert ! appsink";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Could not grab frame." << std::endl;
            break;
        }

        cv::imshow("Camera Feed", frame);
        if (cv::waitKey(30) >= 0) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}