#include "camera/OpenCVCamera.h"
#include <iostream>

OpenCVCamera::OpenCVCamera() {
    cap_.open(0);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    if (!cap_.isOpened())
        std::cerr << "Camera open failed\n";
}

bool OpenCVCamera::capture(cv::Mat& frame) {
    cap_ >> frame;
    return !frame.empty();
}
