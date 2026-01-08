#ifndef OPENCV_CAMERA_H
#define OPENCV_CAMERA_H

#include "CameraInterface.h"

class OpenCVCamera : public CameraInterface {
public:
    void capture() override;
    bool isReady() override;
};

#endif