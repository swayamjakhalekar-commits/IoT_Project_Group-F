#include "perception/TrackDetector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>

bool TrackDetector::process(const cv::Mat& frame,
                            double& lateral_error,
                            double& heading_error)
{
    if (frame.empty())
        return false;

    int h = frame.rows;
    int w = frame.cols;

    // ------------------------------------
    // 1. Region of Interest (lower area)
    // ------------------------------------
    int roi_y = static_cast<int>(h * 0.15);
    int roi_h = h - roi_y;

    cv::Mat roi = frame(cv::Rect(0, roi_y, w, roi_h)).clone();

    // ------------------------------------
    // 2. Convert to HSV
    // ------------------------------------
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // ------------------------------------
    // 3. Track segmentation (IMPROVED)
    // ------------------------------------
    cv::Mat binary;

    // Track = low saturation + mid/low brightness
    cv::inRange(
        hsv,
        cv::Scalar(0, 0, 40),     // allow mid brightness
        cv::Scalar(180, 80, 160), // reject red/white blocks
        binary
    );

    // ------------------------------------
    // 4. Morphological cleanup
    // ------------------------------------
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(7, 7));

    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    // ------------------------------------
    // 5. Debug window (HUMANS ONLY)
    // ------------------------------------
    cv::imshow("Binary Debug", binary);

    // ------------------------------------
    // 6. Center extraction (single scanline)
    // ------------------------------------
    int scan_y = binary.rows - 25;
    std::vector<int> xs;

    for (int x = 0; x < binary.cols; x++) {
        if (binary.at<uchar>(scan_y, x) > 0)
            xs.push_back(x);
    }

    if (xs.size() < 50)
        return false;

    double mean_x = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
    double center_x = w / 2.0;

    // Normalize to [-1, 1]
    lateral_error = (mean_x - center_x) / center_x;
    heading_error = 0.0;  // will be added later

    return true;
}
