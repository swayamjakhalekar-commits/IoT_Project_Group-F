#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>

#include "camera/OpenCVCamera.h"
#include "perception/TrackDetector.h"
#include "control/Controller.h"
#include "safety/SafetySupervisor.h"
#include "system/SharedState.h"
#include "system/TimeUtils.h"

SharedState shared;
std::atomic<bool> running{true};

/* =========================
   Dummy BLE Sender
   ========================= */
void sendBLE(const ControlCommand& cmd)
{
    std::cout << "[BLE] steer=" << cmd.steering
              << " speed=" << cmd.speed << std::endl;
}

/* =========================
   Vision Thread
   ========================= */
void visionThread()
{
    OpenCVCamera cam;
    TrackDetector detector;

    auto last = std::chrono::steady_clock::now();

    while (running)
    {
        cv::Mat frame;

        if (!cam.capture(frame))
            continue;

        // Capture timestamp immediately
        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.t_capture_ns = TimeUtils::nowNs();
        }

        // FPS calculation
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.fps = (dt > 0.0) ? (1.0 / dt) : 0.0;
        }

        // Run perception (TrackDetector handles perception_valid)
        bool ok = detector.process(frame, shared);

        /* ---------- DEBUG VIS ---------- */
        int h = frame.rows;
        int w = frame.cols;

        cv::line(frame,
                 {w / 2, h / 2},
                 {w / 2, h},
                 {255, 0, 0},
                 2);

        if (ok)
        {
            double lat;

            {
                std::lock_guard<std::mutex> lock(shared.mtx);
                lat = shared.lateral_error;
            }

            int cx = static_cast<int>(w / 2 - lat);
            cv::circle(frame, {cx, h - 40}, 6, {0, 0, 255}, -1);
        }

        cv::imshow("Camera Debug", frame);
        cv::waitKey(1);
    }
}

/* =========================
   Control Thread
   ========================= */
void controlThread()
{
    Controller controller;
    SafetySupervisor safety;

    while (running)
    {
        ControlCommand cmd{0.0, 0.0};

        bool valid = false;

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            valid = shared.perception_valid;
        }

        if (!valid)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        cmd = controller.compute(shared);

        safety.enforceSafety(cmd, shared);

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.cmd = cmd;
            shared.t_control_ns = TimeUtils::nowNs();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

/* =========================
   BLE Thread
   ========================= */
void bleThread()
{
    while (running)
    {
        ControlCommand cmd;

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            cmd = shared.cmd;
        }

        sendBLE(cmd);

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.t_ble_ns = TimeUtils::nowNs();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

/* =========================
   Main
   ========================= */
int main()
{
    std::thread t1(visionThread);
    std::thread t2(controlThread);
    std::thread t3(bleThread);

    std::cout << "System running. Press ENTER to stop.\n";
    std::cin.get();

    running = false;

    t1.join();
    t2.join();
    t3.join();

    return 0;
}
