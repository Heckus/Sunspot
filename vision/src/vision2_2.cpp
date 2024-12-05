#include <raspicam/raspicam.h>
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    raspicam::RaspiCam Camera; // Camera object

    // Open camera
    if (!Camera.open()) {
        std::cerr << "Error opening camera" << std::endl;
        return -1;
    }

    // Wait a while until camera stabilizes
    std::cout << "Sleeping for 3 seconds..." << std::endl;
    sleep(3);

    // Create an OpenCV window
    cv::namedWindow("RaspiCam", cv::WINDOW_AUTOSIZE);

    while (true) {
        // Capture
        Camera.grab();

        // Allocate memory
        unsigned char *data = new unsigned char[Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB)];

        // Extract the image in RGB format
        Camera.retrieve(data, raspicam::RASPICAM_FORMAT_RGB);

        // Convert to OpenCV Mat
        cv::Mat image(Camera.getHeight(), Camera.getWidth(), CV_8UC3, data);

        // Display the image
        cv::imshow("RaspiCam", image);

        // Free memory
        delete[] data;

        // Wait for 1 ms and break the loop if 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Close camera
    Camera.release();

    return 0;
}