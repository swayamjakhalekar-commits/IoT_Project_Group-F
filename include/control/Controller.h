#pragma once

struct ControlCommand {
    double steering;
    double speed;
};

class Controller {
public:
    ControlCommand compute(double lateral_error,
                           double heading_error,
                           double fps);
};
