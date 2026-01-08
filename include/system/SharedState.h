#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <mutex>

class SharedState {
public:
    void setSpeed(double s) {
        std::lock_guard<std::mutex> lock(mtx);
        speed = s;
    }
    double getSpeed() {
        std::lock_guard<std::mutex> lock(mtx);
        return speed;
    }
private:
    double speed = 0.0;
    std::mutex mtx;
};

#endif