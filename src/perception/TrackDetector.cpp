#include "perception/TrackDetector.h"
#include <opencv2/opencv.hpp>
#include <vector>
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

    /* =========================
       1. ROI (bottom-biased)
       ========================= */
    int roi_y = static_cast<int>(h * 0.10);
    int roi_h = h - roi_y;
    cv::Mat roi = frame(cv::Rect(0, roi_y, w, roi_h)).clone();

    /* =========================
       2. Preprocessing
       ========================= */
    cv::Mat gray, blur, binary;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);

    // Adaptive threshold handles lighting + red/white blocks
    cv::adaptiveThreshold(
        blur, binary,
        255,
        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY_INV,
        31,
        5
    );

    // Close gaps inside the road
    cv::morphologyEx(
        binary, binary,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, {9,9})
    );

    /* =========================
       3. Keep ONLY the road blob
       ========================= */
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
        return false;

    // Find largest contour = road
    size_t largest_idx = 0;
    double max_area = 0.0;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            largest_idx = i;
        }
    }

    // Mask only the road
    cv::Mat road_mask = cv::Mat::zeros(binary.size(), CV_8UC1);
    cv::drawContours(road_mask, contours,
                     static_cast<int>(largest_idx),
                     cv::Scalar(255), cv::FILLED);

    /* =========================
       4. Scanline-based width detection
       ========================= */
    std::vector<int> centers;

    int scan_start = static_cast<int>(roi_h * 0.45);
    int scan_end   = static_cast<int>(roi_h * 0.85);

    for (int y = scan_start; y < scan_end; y += 5) {
        int left = -1, right = -1;

        for (int x = 0; x < w; x++) {
            if (road_mask.at<uchar>(y, x) > 0) {
                left = x;
                break;
            }
        }

        for (int x = w - 1; x >= 0; x--) {
            if (road_mask.at<uchar>(y, x) > 0) {
                right = x;
                break;
            }
        }

        if (left >= 0 && right > left) {
            int center = (left + right) / 2;
            centers.push_back(center);
        }
    }

    if (centers.size() < 5)
        return false;

    /* =========================
       5. Compute errors
       ========================= */
    int image_center = w / 2;
    int near_center  = centers.back();
    int far_center   = centers.front();

    lateral_error = static_cast<double>(image_center - near_center);

    heading_error = std::atan2(
        static_cast<double>(near_center - far_center),
        static_cast<double>(scan_end - scan_start)
    );

    /* =========================
       6. Debug visualization
       ========================= */
    cv::Mat debug;
    cv::cvtColor(road_mask, debug, cv::COLOR_GRAY2BGR);

    for (int y = scan_start; y < scan_end; y += 5) {
        int idx = (y - scan_start) / 5;
        if (idx < static_cast<int>(centers.size()))
            cv::circle(debug, {centers[idx], y}, 2, {0,255,0}, -1);
    }

    cv::line(debug,
             {far_center, scan_start},
             {near_center, scan_end},
             {0,255,255}, 2);

    cv::imshow("Binary Debug", debug);

    return true;
}
