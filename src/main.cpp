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

        double lat, head;
        bool ok = detector.process(frame, lat, head);

        std::lock_guard<std::mutex> lock(shared.mtx);
        shared.lateral_error = lat;
        shared.heading_error = head;
        shared.fps = 1.0 / dt;
        shared.perception_valid = ok;

        cv::imshow("Camera", frame);
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
