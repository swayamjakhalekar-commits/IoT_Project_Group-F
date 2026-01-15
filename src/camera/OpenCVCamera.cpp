#include "camera/OpenCVCamera.h"
#include <stdexcept>

OpenCVCamera::OpenCVCamera() {
    cap_.open(0, cv::CAP_V4L2);   // /dev/video0 (Cam Link)

    if (!cap_.isOpened()) {
        throw std::runtime_error("Failed to open Cam Link 4K (/dev/video0)");
    }

    // Force safe embedded settings
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, 30);

    // Optional: reduce internal buffering (important for latency)
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
}

bool OpenCVCamera::capture(cv::Mat& frame) {
    return cap_.read(frame);
}
