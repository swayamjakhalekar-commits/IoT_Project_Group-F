#pragma once

#include <opencv2/opencv.hpp>
#include "system/SharedState.h"

class TrackDetector {
public:
    /**
     * @brief Process a camera frame and update perception state.
     *
     * @param frame        Input camera frame
     * @param shared_state Shared system state (errors + timestamps)
     * @return true if perception succeeded, false otherwise
     */
    bool process(const cv::Mat& frame, SharedState& shared_state);
};
