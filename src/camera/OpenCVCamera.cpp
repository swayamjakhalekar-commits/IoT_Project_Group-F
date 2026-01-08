#include "camera/OpenCVCamera.h"
#include <iostream>

void OpenCVCamera::capture() {
    std::cout << "Capturing image with OpenCV" << std::endl;
}

bool OpenCVCamera::isReady() {
    return true;
}