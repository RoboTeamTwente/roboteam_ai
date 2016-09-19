#pragma once

#include "ros/ros.h"

#include "roboteam_vision/DetectionFrame.h"


class WorldDummy {

public:
    WorldDummy();
    void detectionCallback(const roboteam_vision::DetectionFrame msg);

};
