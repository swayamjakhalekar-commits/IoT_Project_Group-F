#include "perception/TrackDetector.h"
#include <opencv2/opencv.hpp>

bool TrackDetector::process(const cv::Mat& frame,
                            double& lateral_error,
                            double& heading_error) {

    int h = frame.rows;
    int w = frame.cols;

    // ROI: bottom half
    int roi_y = static_cast<int>(h * 0.10);
    int roi_h = h - roi_y;
    
    cv::Mat roi = frame(cv::Rect(0, roi_y, w, roi_h)).clone();

    cv::Mat gray, blur, binary;

    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);

    // ---- TEMPORARY THRESHOLD ----
    cv::threshold(blur, binary, 120, 255, cv::THRESH_BINARY_INV);

    // SHOW WHAT DETECTOR SEES
    cv::imshow("Binary Debug", binary);

    // ---- FOR NOW: RETURN FALSE ----
    return false;
}
