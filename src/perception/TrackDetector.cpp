#include "perception/TrackDetector.h"
#include "system/TimeUtils.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <mutex>

bool TrackDetector::process(const cv::Mat& frame,
                            SharedState& shared_state)
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
    cv::Mat gray, blur, equalized, edges;

    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);
    cv::equalizeHist(blur, equalized);
    cv::Canny(equalized, edges, 80, 160);

    cv::morphologyEx(
        edges,
        edges,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, {9,9})
    );

    /* =========================
       3. Find contours
       ========================= */
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat edges_copy = edges.clone();

    cv::findContours(edges_copy,
                     contours,
                     cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
        std::lock_guard<std::mutex> lock(shared_state.mtx);
        shared_state.perception_valid = false;
        return false;
    }

    /* =========================
       4. Largest contour
       ========================= */
    size_t largest_idx = 0;
    double max_area = 0.0;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > max_area)
        {
            max_area = area;
            largest_idx = i;
        }
    }

    cv::Moments m = cv::moments(contours[largest_idx]);

    if (m.m00 == 0)
    {
        std::lock_guard<std::mutex> lock(shared_state.mtx);
        shared_state.perception_valid = false;
        return false;
    }

    int track_cx = static_cast<int>(m.m10 / m.m00);

    /* =========================
       5. Fit line for heading
       ========================= */
    std::vector<cv::Point2f> track_points;
    for (const auto& pt : contours[largest_idx])
        track_points.emplace_back(pt.x, pt.y);

    if (track_points.size() < 4)
    {
        std::lock_guard<std::mutex> lock(shared_state.mtx);
        shared_state.perception_valid = false;
        return false;
    }

    cv::Vec4f line;
    cv::fitLine(track_points, line,
                cv::DIST_L2, 0, 0.01, 0.01);

    float vx = line[0];
    float vy = line[1];

    /* =========================
       6. Compute errors
       ========================= */
    int image_center = w / 2;
    double lateral_error = static_cast<double>(image_center - track_cx);
    double heading_error = std::atan2(vy, vx);

    /* =========================
       7. Update shared state
       ========================= */
    {
        std::lock_guard<std::mutex> lock(shared_state.mtx);
        shared_state.lateral_error = lateral_error;
        shared_state.heading_error = heading_error;
        shared_state.t_perception_ns = TimeUtils::nowNs();
        shared_state.perception_valid = true;
    }

    return true;
}
