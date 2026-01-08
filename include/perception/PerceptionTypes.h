#ifndef PERCEPTION_TYPES_H
#define PERCEPTION_TYPES_H

#include <vector>

struct Point {
    double x, y;
};

struct Track {
    std::vector<Point> points;
};

#endif