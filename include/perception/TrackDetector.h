#ifndef TRACK_DETECTOR_H
#define TRACK_DETECTOR_H

#include "PerceptionTypes.h"

class TrackDetector {
public:
    virtual ~TrackDetector() = default;
    virtual Track detectTrack();
};

#endif