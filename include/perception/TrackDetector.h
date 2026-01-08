#pragma once
#include <opencv2/opencv.hpp>

class TrackDetector {
public:
    bool process(const cv::Mat& frame,
                 double& lateral_error,
                 double& heading_error);
};
