#include "perception/TrackDetector.h"

bool TrackDetector::process(const cv::Mat& frame,
                            double& lateral_error,
                            double& heading_error) {

    // Dummy logic for now (replace later)
    lateral_error = 0.1;
    heading_error = 0.05;
    return true;
}
