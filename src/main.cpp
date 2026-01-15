#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>

#include "camera/OpenCVCamera.h"
#include "perception/TrackDetector.h"
#include "control/Controller.h"
#include "safety/SafetySupervisor.h"
#include "system/SharedState.h"
#include "system/TimeUtils.h"

// Dummy BLE sender
void sendBLE(const ControlCommand& cmd) {
    std::cout << "[BLE] steer=" << cmd.steering
              << " speed=" << cmd.speed << std::endl;
}

SharedState shared;
std::atomic<bool> running{true};

/* =========================
   Camera + Perception Thread
   ========================= */
void visionThread() {
    OpenCVCamera cam;
    TrackDetector detector;

    auto last = std::chrono::steady_clock::now();

    while (running) {
        cv::Mat frame;
        if (!cam.capture(frame))
            continue;

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        double lat = 0.0, head = 0.0;
        bool ok = detector.process(frame, lat, head);

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.frame = frame;
            shared.fps = (dt > 0.0) ? (1.0 / dt) : 0.0;
            shared.perception_valid = ok;
            shared.t_capture_ns = TimeUtils::nowNs();

            if (ok) {
                shared.lateral_error = lat;
                shared.heading_error = head;
            }
        }

        /* ---------- DEBUG VIS ---------- */
        int h = frame.rows;
        int w = frame.cols;

        cv::line(frame,
                 {w / 2, h / 2},
                 {w / 2, h},
                 {255, 0, 0},
                 2);

        if (ok) {
