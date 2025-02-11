#ifndef FRAME_QUEUE_H
#define FRAME_QUEUE_H
#include <queue>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
// Class declaration for FrameQueue
class FrameQueue {
public:
    // Constructor/Destructor declarations
    FrameQueue();
    ~FrameQueue();

    void enqueue(cv::UMat frame);
    cv::UMat dequeue();
    void stop();

    private:
    std::queue<cv::UMat> frameQueue;
    std::mutex mutex;
    std::condition_variable condition;
    bool running = true;
};

#endif // FRAME_QUEUE_H



