#include <opencv2/opencv.hpp>
#include <iostream>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <string.h>
#include <unistd.h>

bool running = true;

struct Buffer {
    void* start;
    size_t length;
};

void signalHandler(int signum) {
    running = false;
}

static int xioctl(int fd, int request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}

class V4L2Camera {
private:
    int fd;
    Buffer* buffers;
    unsigned int n_buffers;
    
public:
    V4L2Camera(const char* device) {
        fd = open(device, O_RDWR);
        if (fd < 0) {
            perror("Cannot open device");
            exit(EXIT_FAILURE);
        }
        initDevice();
    }

    ~V4L2Camera() {
        stopCapture();
        for (unsigned int i = 0; i < n_buffers; ++i) {
            munmap(buffers[i].start, buffers[i].length);
        }
        delete[] buffers;
        close(fd);
    }

    void initDevice() {
        struct v4l2_capability cap;
        struct v4l2_format fmt;

        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            perror("VIDIOC_QUERYCAP");
            exit(EXIT_FAILURE);
        }

        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = 640;
        fmt.fmt.pix.height = 480;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            perror("VIDIOC_S_FMT");
            exit(EXIT_FAILURE);
        }

        initMmap();
    }

    void initMmap() {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            perror("VIDIOC_REQBUFS");
            exit(EXIT_FAILURE);
        }

        buffers = new Buffer[req.count];
        n_buffers = req.count;

        for (unsigned int i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
                perror("VIDIOC_QUERYBUF");
                exit(EXIT_FAILURE);
            }

            buffers[i].length = buf.length;
            buffers[i].start = mmap(NULL, buf.length,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED,
                                  fd, buf.m.offset);

            if (buffers[i].start == MAP_FAILED) {
                perror("mmap");
                exit(EXIT_FAILURE);
            }
        }
    }

    void startCapture() {
        for (unsigned int i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                perror("VIDIOC_QBUF");
                exit(EXIT_FAILURE);
            }
        }

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            perror("VIDIOC_STREAMON");
            exit(EXIT_FAILURE);
        }
    }

    void stopCapture() {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
            perror("VIDIOC_STREAMOFF");
            exit(EXIT_FAILURE);
        }
    }

    cv::Mat captureFrame() {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            perror("VIDIOC_DQBUF");
            return cv::Mat();
        }

        cv::Mat yuyv(480, 640, CV_8UC2, buffers[buf.index].start);
        cv::Mat bgr;
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);

        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            return cv::Mat();
        }

        return bgr;
    }
};

int main() {
    signal(SIGINT, signalHandler);

    V4L2Camera camera("/dev/video0");
    camera.startCapture();

    cv::CascadeClassifier ball_cascade;
    if (!ball_cascade.load("/home/hecke/Code/GitRepositories/Sunspot/vision/models/cascade4.xml")) {
        std::cout << "Error loading face cascade classifier" << std::endl;
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
                frame = camera.captureFrame();
                if (frame.empty()) break;

                cv::imshow("Webcam", frame);
                if (cv::waitKey(1) == 27) break;
            }
            cv::destroyAllWindows();
        }
        else if (input == '2') {
            while (running) {
                frame = camera.captureFrame();
                if (frame.empty()) {
                    std::cout << "Camera disconnected or frame capture failed" << std::endl;
                    return -1;
                }

                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Rect> balls;
                ball_cascade.detectMultiScale(gray, balls, 1.1, 4);

                for (const auto& face : balls) {
                    cv::rectangle(frame, face, cv::Scalar(0, 0, 255), 2);
                    cv::putText(frame, "ball", 
                              cv::Point(face.x, face.y - 5),
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
                std::cout << "Error opening video file" << std::endl;
                continue;
            }

            while (running) {
                cap >> frame;
                if (frame.empty()) break;

                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Rect> balls;
                ball_cascade.detectMultiScale(gray, balls, 1.1, 4);

                for (const auto& face : balls) {
                    cv::rectangle(frame, face, cv::Scalar(0, 0, 255), 2);
                    cv::putText(frame, "ball", 
                              cv::Point(face.x, face.y - 5),
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

    return 0;
}