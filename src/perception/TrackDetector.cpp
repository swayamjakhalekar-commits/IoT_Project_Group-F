#include "perception/TrackDetector.h"
#include <opencv2/opencv.hpp>
#include <numeric>
#include <cmath>

bool TrackDetector::process(const cv::Mat& frame,
                            double& lateral_error,
                            double& heading_error)
{
    if (frame.empty())
        return false;

    const int h = frame.rows;
    const int w = frame.cols;

    /* =======================
       1. ROI (bottom region)
       ======================= */
    int roi_y = static_cast<int>(h * 0.15);
    int roi_h = h - roi_y;
    cv::Mat roi = frame(cv::Rect(0, roi_y, w, roi_h)).clone();

    /* =======================
       2. Preprocessing
       ======================= */
    cv::Mat gray, blur, binary;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);

    cv::threshold(blur, binary, 120, 255, cv::THRESH_BINARY_INV);

    // Remove noise
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, {5,5}));

    /* =======================
       3. Centroid extraction
       ======================= */
    std::vector<int> near_x, far_x;

    int near_y = static_cast<int>(roi_h * 0.80); // close to car
    int far_y  = static_cast<int>(roi_h * 0.40); // look-ahead

    for (int x = 0; x < w; x++) {
        if (binary.at<uchar>(near_y, x) > 0)
            near_x.push_back(x);
        if (binary.at<uchar>(far_y, x) > 0)
            far_x.push_back(x);
    }

    if (near_x.empty() || far_x.empty())
        return false;

    int near_center = std::accumulate(near_x.begin(), near_x.end(), 0) / near_x.size();
    int far_center  = std::accumulate(far_x.begin(),  far_x.end(),  0) / far_x.size();

    int image_center = w / 2;

    /* =======================
       4. Errors
       ======================= */
    lateral_error = static_cast<double>(image_center - near_center);

    heading_error = std::atan2(
        static_cast<double>(near_center - far_center),
        static_cast<double>(near_y - far_y)
    );

    /* =======================
       5. Debug visualization
       ======================= */
    cv::cvtColor(binary, binary, cv::COLOR_GRAY2BGR);

    cv::circle(binary, {near_center, near_y}, 6, {0,255,0}, -1);
    cv::circle(binary, {far_center,  far_y},  6, {255,0,0}, -1);
    cv::line(binary,
             {near_center, near_y},
             {far_center,  far_y},
             {0,255,255}, 2);

    cv::imshow("Binary Debug", binary);

    return true;
}
