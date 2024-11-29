#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <stdint.h>
#include <iostream>
#include <vector>

#define LEARNING_RATE -1

const int fps = 60;
const double scale_factor = 0.5; // Scale factor to shrink the frames

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <video_file>" << std::endl;
        return -1;
    }

    std::string video_file = argv[1];
    cv::Mat frame, resized_frame, motion_mask, background;
    cv::VideoCapture vid(video_file);

    if (!vid.isOpened()) {
        std::cerr << "Error: Could not open video file " << video_file << std::endl;
        return -1;
    }

    cv::Ptr<cv::BackgroundSubtractor> bg_subtractor = cv::createBackgroundSubtractorMOG2();

    // Set up the detector with default parameters.
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();

    while (true) {
        if (!vid.read(frame)) {
            break;
        }

        cv::resize(frame, resized_frame, cv::Size(), scale_factor, scale_factor);

        // Apply MOG
        bg_subtractor->apply(resized_frame, motion_mask, LEARNING_RATE);
        // Get the background image
        bg_subtractor->getBackgroundImage(background);

        // Detect blobs.
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(resized_frame, keypoints);

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        cv::Mat im_with_keypoints;
        cv::drawKeypoints(resized_frame, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Display the resized frame, motion mask, and background image
        cv::imshow("Resized Frame", resized_frame);
        cv::imshow("Motion Mask", motion_mask);
        cv::imshow("Keypoints", im_with_keypoints);
        if (!background.empty()) {
            cv::imshow("Background", background);
        }

        if (cv::waitKey(1) >= 0) {
            break;
        }
    }

    return 0;
}