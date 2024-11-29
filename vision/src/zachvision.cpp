#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <math.h>

const int fps = 60;
const double scale_factor = 0.5; // Scale factor to shrink the frames
const double min_contour_area = 40.0; // Minimum contour area to be considered
const double trail_fade_factor = 0.95; // Factor to fade the trail

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <video_file>" << std::endl;
        return -1;
    }

    std::string video_file = argv[1];
    cv::Mat frame, trail_image;
    cv::VideoCapture vid(video_file);

    if (!vid.isOpened()) {
        std::cerr << "Error: Could not open video file " << video_file << std::endl;
        return -1;
    }

    // Adjusted HSV color range for yellow
    cv::Scalar yellowLower(10, 10, 10);
    cv::Scalar yellowUpper(40, 255, 255);

    while (true) {
        if (!vid.read(frame)) {
            break;
        }

        cv::resize(frame, frame, cv::Size(), scale_factor, scale_factor);

        // Initialize the trail image if it is empty
        if (trail_image.empty()) {
            trail_image = cv::Mat::zeros(frame.size(), frame.type());
        }

        cv::Mat blurred, hsv, mask;
        cv::GaussianBlur(frame, blurred, cv::Size(15, 15), 0); // Increased blur size
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, yellowLower, yellowUpper, mask);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2); // Adjusted iterations
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2); // Adjusted iterations

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Point center;
        if (!contours.empty()) {
            auto c = *std::max_element(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

            if (cv::contourArea(c) > min_contour_area) {
                cv::Moments M = cv::moments(c);
                if (M.m00 != 0) {
                    center = cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
                    cv::circle(trail_image, center, 5, cv::Scalar(0, 255, 0), -1); // Draw on the trail image

                    cv::Point2f enclosing_center;
                    float radius;
                    cv::minEnclosingCircle(c, enclosing_center, radius);

                    if (radius > 10) {
                        cv::circle(frame, enclosing_center, static_cast<int>(radius), cv::Scalar(0, 255, 255), 5);
                    }
                }
            }
        }

        // Fade the trail image
        trail_image *= trail_fade_factor;

        // Blend the trail image with the current frame
        cv::addWeighted(frame, 1.0, trail_image, 0.5, 0, frame);

        cv::imshow("Frame", frame);
        // cv::imshow("HSV", hsv); // Display the HSV image for debugging
        // cv::imshow("Blurred", blurred); // Display the blurred image for debugging
        // cv::imshow("Mask", mask); // Display the mask for debugging

        if (cv::waitKey(1) >= 0) {  // Reduced wait time to 1ms
            break;
        }
        }

    vid.release();
    cv::destroyAllWindows();

    return 0;
}