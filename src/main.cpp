#include <thread>
#include <chrono>
#include "camera/OpenCVCamera.h"
#include "perception/TrackDetector.h"
#include "control/Controller.h"
#include "system/SharedState.h"

SharedState shared;

void visionThread() {
    OpenCVCamera cam;
    TrackDetector detector;

    auto last = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat frame;
        if (!cam.capture(frame)) continue;

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        double lat = 0.0, head = 0.0;
        bool ok = detector.process(frame, lat, head);

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.fps = (dt > 0.0) ? (1.0 / dt) : 0.0;
            shared.perception_valid = ok;

            if (ok) {
                shared.lateral_error = lat;
                shared.heading_error = head;
            }
        }

        // ---------- DEBUG OVERLAY ----------
        int h = frame.rows;
        int w = frame.cols;

        // ROI (bottom half)
        cv::Rect roi(0, h / 2, w, h / 2);
        cv::rectangle(frame, roi, cv::Scalar(0, 255, 0), 2);

        // Image center
        cv::line(frame,
                 cv::Point(w / 2, h / 2),
                 cv::Point(w / 2, h),
                 cv::Scalar(255, 0, 0),
                 2);

        if (ok) {
            int cx = static_cast<int>(w / 2 + lat * (w / 2));
            cv::circle(frame,
                       cv::Point(cx, h - 40),
                       6,
                       cv::Scalar(0, 0, 255),
                       -1);
        }

        cv::imshow("Camera Debug", frame);
        cv::waitKey(1);
    }
}

void controlThread() {
    Controller ctrl;

    while (true) {
        ControlCommand cmd{0,0};

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            if (shared.perception_valid)
                cmd = ctrl.compute(shared.lateral_error,
                                   shared.heading_error,
                                   shared.fps);
        }

        // BLE will be called here later
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}

int main() {
    std::thread t1(visionThread);
    std::thread t2(controlThread);

    t1.join();
    t2.join();
}
