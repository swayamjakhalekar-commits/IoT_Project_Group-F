#include "camera/OpenCVCamera.h"
#include "system/TimeUtils.h"
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
        double init_time = std::chrono::duration<double>(init_end - init_start).count();
        
        std::cout << "    ✓ Camera initialized in " << std::fixed << std::setprecision(3) 
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
            double frame_time = std::chrono::duration<double>(frame_end - frame_start).count() * 1000;
            
            if (success && !frame.empty()) {
                successful_frames++;
                total_capture_time += frame_time;
                
                std::string resolution = std::to_string(frame.cols) + "x" + std::to_string(frame.rows);
                std::cout << std::left << std::setw(8) << i 
                          << std::setw(15) << resolution 
                          << std::setw(12) << std::fixed << std::setprecision(2) << frame_time
                          << "✓\n";
                
                if (i == 0) {
                    first_frame = frame.clone();
                }
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
        std::cout << "    Successful frames: " << successful_frames << "/10\n";
        std::cout << "    Average capture time: " << std::fixed << std::setprecision(2) 
                  << (total_capture_time / successful_frames) << " ms\n";
        std::cout << "    Estimated FPS: " << std::fixed << std::setprecision(1) 
                  << (1000.0 / (total_capture_time / successful_frames)) << " fps\n\n";

        if (!first_frame.empty()) {
            std::cout << "[4/5] Frame validation\n";
            std::cout << "    Width: " << first_frame.cols << " pixels";
            if (first_frame.cols == 1280) {
                std::cout << " ✓\n";
            } else {
                std::cout << " ✗ (expected 1280)\n";
            }
            
            std::cout << "    Height: " << first_frame.rows << " pixels";
            if (first_frame.rows == 720) {
                std::cout << " ✓\n";
            } else {
                std::cout << " ✗ (expected 720)\n";
            }
            
            std::cout << "    Channels: " << first_frame.channels();
            if (first_frame.channels() == 3) {
                std::cout << " (BGR) ✓\n";
            } else {
                std::cout << " ✗ (expected 3)\n";
            }
            
            std::cout << "    Data type: " << first_frame.type();
            if (first_frame.type() == CV_8UC3) {
                std::cout << " (uint8) ✓\n\n";
            } else {
                std::cout << " ✗\n\n";
            }

            std::cout << "[5/5] Saving test frame...\n";
            std::string frame_path = "camera_test_frame.jpg";
            if (cv::imwrite(frame_path, first_frame)) {
                std::cout << "    ✓ Frame saved: " << frame_path << "\n";
            } else {
                std::cerr << "    ✗ Failed to save frame\n";
            }
        }

        std::cout << "\n====================================\n";
        std::cout << "  ✓ CAMERA TEST PASSED\n";
        std::cout << "====================================\n";
        std::cout << "\nNext steps:\n";
        std::cout << "  1. Copy frame to Windows: scp iot@192.168.88.254:camera_test_frame.jpg .\n";
        std::cout << "  2. Test TrackDetector: ./test_detector\n";
        std::cout << "  3. Visualize edge detection: ./visualize\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "\n✗ CAMERA TEST FAILED\n";
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << "\nTroubleshooting:\n";
        std::cerr << "  1. Check camera: lsusb\n";
        std::cerr << "  2. Check device: ls -l /dev/video0\n";
        std::cerr << "  3. Check permissions: v4l2-ctl -d 0 --list-formats\n";
        return -1;
    }
}
