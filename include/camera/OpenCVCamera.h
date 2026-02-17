#pragma once

#include <opencv2/opencv.hpp>

class OpenCVCamera
{
public:
    OpenCVCamera();
    ~OpenCVCamera();

    bool capture(cv::Mat& frame);

private:
    cv::VideoCapture cap;
};
