#pragma once

#include "roboteam_vision/DetectionFrame.h"
#include "roboteam_world/World.h"


namespace rtt {

    class WorldBase {

    public:
        WorldBase() {};
        virtual roboteam_world::World as_message() { return roboteam_world::World(); };
        virtual void detection_callback(const roboteam_vision::DetectionFrame msg) {};

    };

}
