#include <opencv2/opencv.hpp>   // Main OpenCV header - includes most common functionality
#include <iostream>

// Image Processing Function
// This function demonstrates common image processing operations
void processFrame(cv::Mat& frame) {
    // Convert the image from BGR (OpenCV default) to grayscale
    // BGR is Blue-Green-Red, similar to RGB but byte-ordered differently
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce image noise
    // Size(5, 5) is the kernel size - must be odd numbers
    // Larger numbers create more blur
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    // Thresholding converts grayscale to binary (black and white)
    // 127 is the threshold value (0-255)
    // 255 is the maximum value
    // Values > 127 become 255 (white), values <= 127 become 0 (black)
    cv::Mat thresh;
    cv::threshold(blurred, thresh, 127, 255, cv::THRESH_BINARY);

    // Contours are the boundaries of shapes in the image
    // vectors of Points where each Point is an (x,y) coordinate
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;  // Stores the relationship between contours
    
    // Find contours in the binary image
    // RETR_TREE retrieves all contours and reconstructs their hierarchy
    // CHAIN_APPROX_SIMPLE compresses the contours to save memory
    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Draw the contours on the original image
    // -1 means draw all contours
    // Scalar(0, 255, 0) is the color (green in BGR)
    // 2 is the thickness of the contour line
    cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 2);
}

// Circle Detection Function
// Particularly useful for detecting round objects like volleyballs
void detectCircles(cv::Mat& frame) {
    // Convert to grayscale for circle detection
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Blur the image to reduce noise
    // Larger kernel size (9,9) used because circle detection is sensitive to noise
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
    
    // Vector to store the detected circles
    // Each circle is stored as a 3-element vector: (x, y, radius)
    std::vector<cv::Vec3f> circles;
    
    // HoughCircles detection
    // Parameters explained:
    // HOUGH_GRADIENT: detection method
    // 1: ratio of resolution
    // gray.rows/8: minimum distance between circle centers
    // 100: upper threshold for edge detection
    // 30: threshold for center detection
    // 20: minimum radius in pixels
    // 100: maximum radius in pixels
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                     gray.rows/8,  // Adjust this value based on how far apart your circles are
                     100, 30,      // Adjust these parameters to fine-tune detection
                     20, 100       // Adjust min/max radius based on your volleyball size
    );
    
    // Draw each detected circle
    for(size_t i = 0; i < circles.size(); i++) {
        // Convert the floating point circle coordinates to integers
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);  // Circle center
        int radius = c[2];                         // Circle radius
        
        // Draw the circle center (small green dot)
        cv::circle(frame, center, 3, cv::Scalar(0,255,0), -1);
        
        // Draw the circle outline (red)
        // -1 for thickness means fill the circle
        cv::circle(frame, center, radius, cv::Scalar(0,0,255), 3);
    }
}

// Object Detection using Deep Learning
// This shows how to use pre-trained neural networks with OpenCV
void objectDetection(cv::Mat& frame) {
    // Load a pre-trained neural network
    // You'll need to provide paths to your model files
    cv::dnn::Net net = cv::dnn::readNet(
        "path_to_weights.weights",  // Model weights
        "path_to_config.cfg"        // Model configuration
    );
    
    // Prepare the image for the neural network
    // Parameters:
    // 1/255.0: scale factor to normalize pixel values to [0,1]
    // Size(416, 416): input size required by the network
    // Scalar(0,0,0): mean subtraction values
    // true: swap RGB to BGR
    // false: don't crop image
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1/255.0, 
        cv::Size(416, 416), cv::Scalar(0,0,0), true, false);
    
    // Set the prepared image as network input
    net.setInput(blob);
    
    // Get the names of the output layers
    // This is needed because we need to know which layers give us the detections
    std::vector<cv::String> outLayerNames;
    std::vector<int> outLayers = net.getUnconnectedOutLayers();
    std::vector<cv::String> layerNames = net.getLayerNames();
    outLayerNames.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i) {
        outLayerNames[i] = layerNames[outLayers[i] - 1];
    }
    
    // Perform forward pass through the network
    // This is where the actual detection happens
    std::vector<cv::Mat> detections;
    net.forward(detections, outLayerNames);
    
    // At this point, you would process the detections based on your model
    // This typically involves:
    // 1. Extracting bounding boxes
    // 2. Getting confidence scores
    // 3. Applying non-maximum suppression
    // 4. Drawing the results
    // The exact implementation depends on your model's output format
}

/* Build and Installation Notes:

Ubuntu/Debian:
-------------
1. Install OpenCV:
   sudo apt update
   sudo apt install libopencv-dev

2. Compile:
   g++ -o webcam_app main.cpp `pkg-config --cflags --libs opencv4`

Windows:
--------
1. Install OpenCV using vcpkg:
   vcpkg install opencv4:x64-windows
   
2. Compile with Visual Studio or:
   g++ -o webcam_app main.cpp -I"C:\opencv\include" -L"C:\opencv\lib" -lopencv_core440 -lopencv_highgui440 -lopencv_imgproc440

macOS:
------
1. Install using Homebrew:
   brew install opencv

2. Compile:
   g++ -o webcam_app main.cpp `pkg-config --cflags --libs opencv4`
*/

// This function will be called from main() to set up all display windows
void setupWindows() {
    // Create windows with specific sizes and positions
    // WINDOW_NORMAL allows us to resize windows manually
    cv::namedWindow("Original Feed", cv::WINDOW_NORMAL);
    cv::namedWindow("Grayscale", cv::WINDOW_NORMAL);
    cv::namedWindow("Processed", cv::WINDOW_NORMAL);
    cv::namedWindow("Detection Results", cv::WINDOW_NORMAL);
    
    // Move windows to specific positions on screen
    // This prevents windows from overlapping
    cv::moveWindow("Original Feed", 0, 0);        // Top-left
    cv::moveWindow("Grayscale", 640, 0);         // Top-right
    cv::moveWindow("Processed", 0, 520);         // Bottom-left
    cv::moveWindow("Detection Results", 640, 520); // Bottom-right
    
    // Optionally resize windows to specific dimensions
    cv::resizeWindow("Original Feed", 640, 480);
    cv::resizeWindow("Grayscale", 640, 480);
    cv::resizeWindow("Processed", 640, 480);
    cv::resizeWindow("Detection Results", 640, 480);
}

// Enhanced version of detectCircles with visualization windows
void enhancedDetectCircles(cv::Mat& frame) {
    // Store a copy of the original frame for display
    cv::Mat originalFrame = frame.clone();
    
    // 1. Convert to grayscale - First processing step
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // Display grayscale version
    cv::imshow("Grayscale", gray);
    
    // 2. Create a processed view that will show all our processing steps
    cv::Mat processed = gray.clone();
    
    // 3. Apply Gaussian blur
    // Create a copy for the processed view
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(9, 9), 2, 2);
    // Show the blurred image in processed window
    cv::imshow("Processed", blurred);
    
    // 4. Detect circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1,
                     blurred.rows/8,  // Minimum distance between circles
                     100, 30,         // Edge detection parameters
                     20, 100          // Min/Max radius
    );
    
    // 5. Create a final detection display
    cv::Mat detectionDisplay = originalFrame.clone();
    
    // Draw detected circles on both the original frame and detection display
    for(size_t i = 0; i < circles.size(); i++) {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        int radius = c[2];
        
        // Draw on detection display
        // Circle center (green dot)
        cv::circle(detectionDisplay, center, 3, cv::Scalar(0,255,0), -1);
        // Circle outline (red)
        cv::circle(detectionDisplay, center, radius, cv::Scalar(0,0,255), 3);
        
        // Add text with circle information
        std::string info = "R=" + std::to_string(radius);
        cv::putText(detectionDisplay, info, 
                    cv::Point(center.x - 20, center.y + radius + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2);
        
        // Draw on original frame for main display
        cv::circle(frame, center, radius, cv::Scalar(0,0,255), 3);
    }
    
    // Display results
    cv::imshow("Detection Results", detectionDisplay);
    cv::imshow("Original Feed", frame);
}

// Enhanced Object Detection with visualization
void enhancedObjectDetection(cv::Mat& frame) {
    // Create copies for different visualization stages
    cv::Mat preprocessed = frame.clone();
    cv::Mat detection_viz = frame.clone();
    
    // 1. Preprocess the image
    // Convert to blob (neural network input format)
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1/255.0, 
        cv::Size(416, 416), cv::Scalar(0,0,0), true, false);
    
    // Display preprocessed image (normalized)
    // We need to convert the blob back to a displayable format
    cv::Mat preprocessed_display;
    std::vector<cv::Mat> channels;
    // Extract the first channel of the blob
    cv::dnn::imagesFromBlob(blob, channels);
    // Normalize for display
    cv::normalize(channels[0], preprocessed_display, 0, 255, cv::NORM_MINMAX);
    cv::imshow("Processed", preprocessed_display);
    
    // 2. Run detection
    cv::dnn::Net net = cv::dnn::readNet("path_to_weights.weights", "path_to_config.cfg");
    net.setInput(blob);
    
    // Get output layers
    std::vector<cv::String> outLayerNames;
    std::vector<int> outLayers = net.getUnconnectedOutLayers();
    std::vector<cv::String> layerNames = net.getLayerNames();
    outLayerNames.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i) {
        outLayerNames[i] = layerNames[outLayers[i] - 1];
    }
    
    // Forward pass
    std::vector<cv::Mat> detections;
    net.forward(detections, outLayerNames);
    
    // 3. Process and visualize detections
    // Create a detection visualization
    cv::Mat detection_results = frame.clone();
    
    // Process each detection
    // (This is a simplified example - actual implementation depends on your model)
    for (const auto& detection : detections) {
        for (int i = 0; i < detection.rows; ++i) {
            float confidence = detection.at<float>(i, 4);
            if (confidence > 0.5) {  // Confidence threshold
                // Get bounding box coordinates
                float x = detection.at<float>(i, 0);
                float y = detection.at<float>(i, 1);
                float width = detection.at<float>(i, 2);
                float height = detection.at<float>(i, 3);
                
                // Draw bounding box
                cv::rectangle(detection_results,
                            cv::Point(x, y),
                            cv::Point(x + width, y + height),
                            cv::Scalar(0, 255, 0), 2);
                
                // Add confidence score
                std::string label = "Confidence: " + 
                                  std::to_string(static_cast<int>(confidence * 100)) + "%";
                cv::putText(detection_results, label,
                          cv::Point(x, y - 10),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5,
                          cv::Scalar(0, 255, 0), 2);
            }
        }
    }
    
    // Display final detection results
    cv::imshow("Detection Results", detection_results);
}

int main(void) {
    // VideoCapture is the OpenCV class for capturing video
    // The argument 0 specifies the first available camera (usually built-in webcam)
    // You can use 1, 2, etc. for additional cameras, or a string for video files ("video.mp4")
    cv::VideoCapture cap(0);
    
    // isOpened() checks if the camera was successfully initialized
    // This is important because the camera might be in use by another program
    // or might not exist
    if (!cap.isOpened()) {
        std::cout << "Error: Could not open camera. Check if:" << std::endl;
        std::cout << "  1. Your camera is connected" << std::endl;
        std::cout << "  2. Another program isn't using it" << std::endl;
        std::cout << "  3. You have permission to access the camera" << std::endl;
        return -1;
    }

    // Set up all our display windows
    setupWindows();
    
    // Optional: Add trackbars for parameter adjustment
    int minRadius = 20;
    int maxRadius = 100;
    cv::createTrackbar("Min Radius", "Detection Results", &minRadius, 100);
    cv::createTrackbar("Max Radius", "Detection Results", &maxRadius, 200);

    while (true) {
        // Mat (Matrix) is OpenCV's basic image container
        // It can store images in various formats (RGB, grayscale, etc.)
        cv::Mat frame;

        // cap.read() captures a new frame from the camera
        // The frame is stored in the Mat object we created
        cap.read(frame);

        // Always check if the frame is valid
        // Frames might be invalid if the camera is disconnected or malfunctioning
        if (frame.empty()) {
            std::cout << "Error: No frame received from camera" << std::endl;
            break;
        }

        // Use our enhanced detection function
        enhancedDetectCircles(frame);
        
        // Wait for 25ms and check if a key was pressed
        // The 25ms wait helps control the frame rate
        // Returns -1 if no key was pressed, or the ASCII value of the pressed key
        char c = (char)cv::waitKey(25);

        // 27 is the ASCII value for the ESC key
        if (c == 27) 
            break;
    }

    // Cleanup
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

