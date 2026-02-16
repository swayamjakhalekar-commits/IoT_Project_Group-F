#pragma once

#include "camera/CameraInterface.h"
#include <opencv2/opencv.hpp>

class OpenCVCamera : public CameraInterface {
public:
    OpenCVCamera();
    bool capture(cv::Mat& frame) override;

private:
    cv::VideoCapture cap_;
};
