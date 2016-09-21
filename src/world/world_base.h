#pragma once

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/World.h"


namespace rtt {

    class WorldBase {

    public:
        WorldBase() {};
        virtual roboteam_msgs::World as_message() { return roboteam_msgs::World(); };
        virtual void detection_callback(const roboteam_msgs::DetectionFrame msg) {};

    };

}
