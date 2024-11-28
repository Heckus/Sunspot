#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <iostream>

#ifdef _WIN32
    #include <windows.h>
#elif __linux__
    #include <libudev.h>
    #include <mntent.h>
#endif

class USBDetector {
private:
    std::string usb_path;

public:
    // Function to detect USB drives on Windows
    #ifdef _WIN32
    bool detectUSBDrive() {
        // Get logical drives bitmask
        DWORD drives = GetLogicalDrives();
        char driveLetter = 'A';

        // Iterate through all possible drive letters
        while (drives) {
            if (drives & 1) {
                char path[4] = {driveLetter, ':', '\\', '\0'};
                
                // Get drive type (USB drives are typically REMOVABLE)
                if (GetDriveTypeA(path) == DRIVE_REMOVABLE) {
                    usb_path = std::string(path);
                    return true;
                }
            }
            drives >>= 1;
            driveLetter++;
        }
        return false;
    }
    #elif __linux__
    bool detectUSBDrive() {
        // Create udev object for device detection
        struct udev* udev = udev_new();
        if (!udev) return false;

        // Read /etc/mtab to find mounted USB devices
        FILE* mtab = setmntent("/etc/mtab", "r");
        struct mntent* entry;

        while ((entry = getmntent(mtab)) != nullptr) {
            // Check if the mount point is a removable device
            if (strstr(entry->mnt_fsname, "/dev/sd") != nullptr) {
                usb_path = entry->mnt_dir;
                endmntent(mtab);
                udev_unref(udev);
                return true;
            }
        }

        endmntent(mtab);
        udev_unref(udev);
        return false;
    }
    #endif

    // Get the detected USB path
    std::string getUSBPath() const {
        return usb_path;
    }
};

class VideoProcessor {
private:
    cv::VideoCapture cap;
    cv::VideoWriter writer;
    std::string output_path;
    bool is_recording;

    // AI model parameters would go here
    // For example: neural network model, configuration, etc.

public:
    VideoProcessor() : is_recording(false) {}

    bool initializeCamera(int camera_id = 0) {
        // Open the default camera (usually built-in webcam)
        cap.open(camera_id);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open camera." << std::endl;
            return false;
        }
        return true;
    }

    bool startRecording(const std::string& usb_path) {
        if (!cap.isOpened()) return false;

        // Generate timestamp for unique filename
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        
        // Create output filename with timestamp
        output_path = usb_path + "/video_" + 
                     std::to_string(timestamp) + ".avi";

        // Get video properties for writer
        int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        double fps = cap.get(cv::CAP_PROP_FPS);

        // Initialize video writer with XVID codec
        writer.open(output_path, 
                   cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
                   fps, 
                   cv::Size(frame_width, frame_height));

        if (!writer.isOpened()) {
            std::cerr << "Error: Could not create video file." << std::endl;
            return false;
        }

        is_recording = true;
        return true;
    }

    void processFrames() {
        cv::Mat frame;
        
        while (is_recording) {
            // Capture frame from camera
            cap.read(frame);
            if (frame.empty()) break;

            // Perform AI object detection (pseudo-code)
            /*
            // 1. Preprocess frame (resize, normalize)
            preprocessFrame(frame);
            
            // 2. Run inference through AI model
            std::vector<Detection> detections = model.detect(frame);
            
            // 3. Draw bounding boxes and labels
            for (const auto& detection : detections) {
                drawBoundingBox(frame, detection);
            }
            */

            // Write frame to video file
            writer.write(frame);

            // Display frame (optional)
            cv::imshow("Video Recording", frame);
            
            // Break loop if 'q' is pressed
            if (cv::waitKey(1) == 'q') break;
        }
    }

    void stopRecording() {
        is_recording = false;
        if (writer.isOpened()) writer.release();
        if (cap.isOpened()) cap.release();
        cv::destroyAllWindows();
    }
};

int main() {
    // Create USB detector instance
    USBDetector usb_detector;

    // Wait for USB drive to be detected
    std::cout << "Waiting for USB drive..." << std::endl;
    while (!usb_detector.detectUSBDrive()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "USB drive detected at: " << usb_detector.getUSBPath() << std::endl;

    // Create video processor instance
    VideoProcessor processor;

    // Initialize camera
    if (!processor.initializeCamera()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    // Start recording to USB drive
    if (!processor.startRecording(usb_detector.getUSBPath())) {
        std::cerr << "Failed to start recording." << std::endl;
        return -1;
    }

    // Process frames until stopped
    processor.processFrames();

    // Clean up
    processor.stopRecording();

    return 0;
}