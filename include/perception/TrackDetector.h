#pragma once

#include <opencv2/opencv.hpp>
#include "system/SharedState.h"

class TrackDetector {
public:
    // Process frame and update shared perception state
    bool process(const cv::Mat& frame, SharedState& shared_state);
};
