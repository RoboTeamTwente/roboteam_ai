#pragma once

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/World.h"

#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"

#include "roboteam_world/world/world_base.h"


namespace rtt {

    class WorldDummy : public WorldBase {
        /**
         * WorldDummy
         * Very naive world manager.
         * Simply forwards anything it gets from the vision.
         * It will take the first ball it gets as the actuall ball.
         */

    private:
        std::vector<rtt::Robot> robots_yellow;
        std::vector<rtt::Robot> robots_blue;
        rtt::Ball ball;

    public:

        WorldDummy() {};

        /**
         * Resets the world using the stored configuration.
         */
        void reset();

        /**
         * Converts this world into a ros message.
         */
        roboteam_msgs::World as_message();

        /**
         * To be called when a detectionframe message is received.
         */
        void detection_callback(const roboteam_msgs::DetectionFrame msg);
    };

}
