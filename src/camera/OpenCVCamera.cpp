#include "camera/OpenCVCamera.h"
#include <stdexcept>
#include <unistd.h>

OpenCVCamera::OpenCVCamera() {
    cap_.open(0, cv::CAP_V4L2);   // /dev/video0 (Elgato Capture)

    if (!cap_.isOpened()) {
        throw std::runtime_error("Failed to open Elgato camera (/dev/video0)");
    }

    // Elgato camera settings
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap_.set(cv::CAP_PROP_FPS, 30);

    // Reduce internal buffering (important for latency)
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    // Give camera time to stabilize
    sleep(1);
}

bool OpenCVCamera::capture(cv::Mat& frame) {
    return cap_.read(frame);
}
