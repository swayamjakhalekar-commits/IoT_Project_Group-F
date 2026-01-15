#pragma once

#include <opencv2/opencv.hpp>
#include "system/SharedState.h"

class CameraInterface {
public:
    virtual ~CameraInterface() = default;

    // Capture frame and update shared timing state
    virtual bool capture(cv::Mat& frame, SharedState& shared_state) = 0;
};

