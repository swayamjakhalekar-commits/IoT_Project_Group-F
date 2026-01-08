#include "perception/TrackDetector.h"
#include <iostream>

Track TrackDetector::detectTrack() {
    std::cout << "Detecting track" << std::endl;
    Track t;
    t.points = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}};
    return t;
}