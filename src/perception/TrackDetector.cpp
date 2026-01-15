#include "perception/TrackDetector.h"
#include <opencv2/opencv.hpp>

bool TrackDetector::process(const cv::Mat& frame,
                            double& lateral_error,
                            double& heading_error) {
    int h = frame.rows;
    int w = frame.cols;

    // ROI: bottom half
    cv::Mat roi = frame(cv::Rect(0, h/2, w, h/2)).clone();

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
