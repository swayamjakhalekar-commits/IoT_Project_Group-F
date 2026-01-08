#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {
public:
    virtual ~Controller() = default;
    virtual void control(double speed, double steering);
};

#endif