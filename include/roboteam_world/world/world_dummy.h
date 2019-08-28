#pragma once

#include <messages_robocup_ssl_detection.pb.h>
#include "World.pb.h"
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
        std::vector<rtt::Robot> yellow;
        std::vector<rtt::Robot> blue;
        rtt::Ball ball;

    public:

        WorldDummy() {};

        /**
         * Resets the world.
         */
        void reset();

        /**
         * Converts this world into a ros message.
         */
        roboteam_proto::World as_message() const;

        /**
         * To be called when a detectionframe message is received.
         */
        void detection_callback(const roboteam_proto::SSL_DetectionFrame msg);
    };

}
