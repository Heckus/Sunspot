// OpenCV GStreamer pipeline to capture and save video
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <csignal>

bool running = true;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

int main() {
    // Register signal handler
    std::signal(SIGINT, signalHandler);

    std::string pipeline = "v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=(int)3840, height=(int)2160 ! nvjpegdec ! video/x-raw, format=I420 ! appsink";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream!" << std::endl;
        return -1;
    }

    cv::VideoWriter writer("output.mp4", cv::VideoWriter::fourcc('X','2','6','4'), 30, cv::Size(1280, 720));

    while (running) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        writer.write(frame);  // Save the frame to the video file
    }

    cap.release();
    writer.release();

    return 0;
}
