#pragma once

#include "camera/CameraInterface.h"
#include "system/SharedState.h"
#include <opencv2/opencv.hpp>

class OpenCVCamera : public CameraInterface {
public:
    OpenCVCamera();

    // Capture frame and update SharedState timing
    bool capture(cv::Mat& frame, SharedState& shared_state) override;

private:
    cv::VideoCapture cap_;
};
