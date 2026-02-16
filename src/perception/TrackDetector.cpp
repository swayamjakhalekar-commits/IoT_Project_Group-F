#include "perception/TrackDetector.h"
#include "system/SharedState.h"
#include "system/TimeUtils.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
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
       2. Preprocessing for edge detection
       ========================= */
    cv::Mat gray, blur, equalized, edges;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    /* =========================
        3. Remove noise
       ========================= */
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);

    /* =========================
       4. Improve contrast
       ========================= */
    cv::equalizeHist(blur, equalized);
    
  /* =========================
       5. Better edge detection
       ========================= */
    cv::Canny(equalized, edges, 80, 160);

    /* =========================
       6. Morphological operations to clean edges
       ========================= */
    cv::morphologyEx(
        edges, edges,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, {9, 9})
    );

    /* =========================
       7. Find contours from edges
       ========================= */
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat edges_copy = edges.clone();
    cv::findContours(edges_copy, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
        return false;

    /* =========================
       8. Find largest contour (main track)
       ========================= */
    size_t largest_idx = 0;
    double max_area = 0.0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            largest_idx = i;
        }
    }

    /* =========================
       9. Get track centerline using moments
       ========================= */
    cv::Moments m = cv::moments(contours[largest_idx]);
    
    if (m.m00 == 0)
        return false;

    int track_cx = static_cast<int>(m.m10 / m.m00);
    int track_cy = static_cast<int>(m.m01 / m.m00);

    /* =========================
       10. Fit line to track edges for heading
       ========================= */
    std::vector<cv::Point2f> track_points;
    for (const auto& pt : contours[largest_idx]) {
        track_points.push_back(cv::Point2f(pt.x, pt.y));
    }

    cv::Vec4f line;
    if (track_points.size() >= 4) {
        cv::fitLine(track_points, line, cv::DIST_L2, 0, 0.01, 0.01);
    } else {
        return false;
    }

    float vx = line[0];
    float vy = line[1];

    /* =========================
       11. Compute lateral and heading errors
       ========================= */
    int image_center = w / 2;
    double lateral_error = static_cast<double>(image_center - track_cx);

    // Heading error: angle of track direction
    double heading_error = std::atan2(vy, vx);

    /* =========================
       12. Update SharedState + timestamp
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
