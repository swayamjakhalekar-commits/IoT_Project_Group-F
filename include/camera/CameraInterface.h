#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

class CameraInterface {
public:
    virtual ~CameraInterface() = default;
    virtual void capture() = 0;
    virtual bool isReady() = 0;
};

#endif