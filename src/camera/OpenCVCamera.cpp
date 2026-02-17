#include "camera/OpenCVCamera.h"

#include <stdexcept>
#include <iostream>
#include <opencv2/opencv.hpp>

OpenCVCamera::OpenCVCamera()
{
    // Open Elgato Cam Link 4K explicitly via V4L2
    if (!cap.open("/dev/video0", cv::CAP_V4L2))
    {
        throw std::runtime_error("Failed to open camera (/dev/video0)");
    }

    // Optional: set resolution (adjust if needed)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    std::cout << "[Camera] Opened /dev/video0 successfully\n";
}

bool OpenCVCamera::capture(cv::Mat& frame)
{
    if (!cap.isOpened())
        return false;

    cap >> frame;

    if (frame.empty())
        return false;

    return true;
}

OpenCVCamera::~OpenCVCamera()
{
    if (cap.isOpened())
        cap.release();
}
