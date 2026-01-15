#include "perception/TrackDetector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>

bool TrackDetector::process(const cv::Mat& frame,
                            double& lateral_error,
                            double& heading_error)
{
    // -------------------------------
    // 1. Basic checks
    // -------------------------------
    if (frame.empty())
        return false;

    int h = frame.rows;
    int w = frame.cols;

    // -------------------------------
    // 2. Region of Interest (ROI)
    //    Adjust height here if needed
    // -------------------------------
    int roi_y = static_cast<int>(h * 0.10);   // ↑ increase height here
    int roi_h = h - roi_y;

    cv::Mat roi = frame(cv::Rect(0, roi_y, w, roi_h)).clone();

    // -------------------------------
    // 3. Convert to HSV
    // -------------------------------
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // -------------------------------
    // 4. Threshold DARK track surface
    //    (Reject red/white boundaries)
    // -------------------------------
    cv::Mat binary;

    cv::inRange(
        hsv,
        cv::Scalar(0, 0, 0),        // low HSV (dark)
        cv::Scalar(180, 255, 90),   // high HSV (dark)
        binary
    );

    // -------------------------------
    // 5. Morphological cleanup
    // -------------------------------
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(5, 5));

    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    // -------------------------------
    // 6. Debug: show binary mask
    // -------------------------------
    cv::imshow("Binary Debug", binary);

    // -------------------------------
    // 7. Compute track center
    //    (scan horizontal slice)
    // -------------------------------
    int scan_y = binary.rows - 20;  // near bottom of ROI
    std::vector<int> track_pixels;

    for (int x = 0; x < binary.cols; x++) {
        if (binary.at<uchar>(scan_y, x) > 0)
            track_pixels.push_back(x);
    }

    // -------------------------------
    // 8. Validate detection
    // -------------------------------
    if (track_pixels.size() < 30) {
        return false;   // not enough track pixels
    }

    // -------------------------------
    // 9. Compute center & error
    // -------------------------------
    double mean_x = std::accumulate(
        track_pixels.begin(),
        track_pixels.end(),
        0.0) / track_pixels.size();

    double center_x = w / 2.0;

    // Normalize error to [-1, 1]
    lateral_error = (mean_x - center_x) / center_x;

    // Heading not implemented yet
    heading_error = 0.0;

    return true;
}
