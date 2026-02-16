#include "camera/OpenCVCamera.h"

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <iomanip>

int main() {
    std::cout << "====================================\n";
    std::cout << "  ELGATO CAMERA MODULE TEST\n";
    std::cout << "====================================\n\n";

    try {
        std::cout << "[1/5] Initializing OpenCVCamera...\n";
        auto init_start = std::chrono::steady_clock::now();
        OpenCVCamera cam;
        auto init_end = std::chrono::steady_clock::now();
        double init_time =
            std::chrono::duration<double>(init_end - init_start).count();

        std::cout << "    ✓ Camera initialized in "
                  << std::fixed << std::setprecision(3)
                  << init_time << "s\n\n";

        std::cout << "[2/5] Testing frame capture (10 frames)...\n";
        std::cout << std::string(50, '-') << "\n";
        std::cout << std::left << std::setw(8) << "Frame"
                  << std::setw(15) << "Resolution"
                  << std::setw(12) << "Time (ms)"
                  << "Status\n";
        std::cout << std::string(50, '-') << "\n";

        double total_capture_time = 0.0;
        cv::Mat first_frame;
        int successful_frames = 0;

        for (int i = 0; i < 10; i++) {
            cv::Mat frame;

            auto frame_start = std::chrono::steady_clock::now();
            bool success = cam.capture(frame);
            auto frame_end = std::chrono::steady_clock::now();

            double frame_time =
                std::chrono::duration<double>(frame_end - frame_start).count() * 1000;

            if (success && !frame.empty()) {
                successful_frames++;
                total_capture_time += frame_time;

                std::string resolution =
                    std::to_string(frame.cols) + "x" +
                    std::to_string(frame.rows);

                std::cout << std::left << std::setw(8) << i
                          << std::setw(15) << resolution
                          << std::setw(12) << std::fixed
                          << std::setprecision(2) << frame_time
                          << "✓\n";

                if (i == 0)
                    first_frame = frame.clone();
            } else {
                std::cout << std::left << std::setw(8) << i
                          << std::setw(15) << "N/A"
                          << std::setw(12) << "---"
                          << "✗ FAILED\n";
            }
        }

        std::cout << std::string(50, '-') << "\n\n";

        if (successful_frames == 0) {
            std::cerr << "✗ NO FRAMES CAPTURED - Camera test FAILED\n";
            return -1;
        }

        std::cout << "[3/5] Capture Statistics\n";
        std::cout << "    Successful frames: "
                  << successful_frames << "/10\n";

        double avg_ms = total_capture_time / successful_frames;
        double fps = (avg_ms > 0.0) ? (1000.0 / avg_ms) : 0.0;

        std::cout << "    Average capture time: "
                  << std::fixed << std::setprecision(2)
                  << avg_ms << " ms\n";

        std::cout << "    Estimated FPS: "
                  << std::fixed << std::setprecision(1)
                  << fps << " fps\n\n";

        if (!first_frame.empty()) {
            std::cout << "[4/5] Frame validation\n";

            std::cout << "    Width: "
                      << first_frame.cols
                      << (first_frame.cols == 1280 ? " ✓\n" : " ✗\n");

            std::cout << "    Height: "
                      << first_frame.rows
                      << (first_frame.rows == 720 ? " ✓\n" : " ✗\n");

            std::cout << "    Channels: "
                      << first_frame.channels()
                      << (first_frame.channels() == 3 ? " ✓\n" : " ✗\n");

            std::cout << "    Data type: "
                      << first_frame.type()
                      << (first_frame.type() == CV_8UC3 ? " ✓\n\n" : " ✗\n\n");

            std::cout << "[5/5] Saving test frame...\n";

            if (cv::imwrite("camera_test_frame.jpg", first_frame))
                std::cout << "    ✓ Frame saved\n";
            else
                std::cerr << "    ✗ Failed to save frame\n";
        }

        std::cout << "\n====================================\n";
        std::cout << "  ✓ CAMERA TEST PASSED\n";
        std::cout << "====================================\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\n✗ CAMERA TEST FAILED\n";
        std::cerr << "Error: " << e.what() << "\n";
        return -1;
    }
}
