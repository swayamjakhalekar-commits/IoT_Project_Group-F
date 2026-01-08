#pragma once
#include "camera/CameraInterface.h"

class OpenCVCamera : public CameraInterface {
public:
    OpenCVCamera();
    bool capture(cv::Mat& frame) override;
private:
    cv::VideoCapture cap_;
};
