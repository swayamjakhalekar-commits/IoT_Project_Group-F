#pragma once

#include <opencv2/opencv.hpp>

class CameraInterface {
public:
    virtual ~CameraInterface() = default;

    // Capture frame only
    virtual bool capture(cv::Mat& frame) = 0;
};
