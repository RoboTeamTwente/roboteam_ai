#pragma once

#include <map>

#include "roboteam_vision/DetectionFrame.h"
#include "roboteam_world/World.h"

#include "../robot.h"
#include "../ball.h"

#include "world_base.h"


namespace rtt {

    typedef std::map<uint, rtt::Robot> RobotMap;


    class WorldDummy : public WorldBase {
        /**
         * WorldDummy
         * Very naive world manager.
         * Simply forwards anything it gets from the vision.
         * It will take the first ball it gets as the actuall ball.
         */

    private:
        RobotMap robots_yellow;
        RobotMap robots_blue;
        rtt::Ball ball;

    public:
        WorldDummy() {};
        void detection_callback(const roboteam_vision::DetectionFrame msg);

        roboteam_world::World as_message();
    };

}
