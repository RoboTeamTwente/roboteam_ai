#pragma once

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/World.h"

#include "roboteam_world/world/world_config.h"


namespace rtt {

    class WorldBase {

    protected:
        // The world configuration.
        // Contains things like the amount of cameras e.d.
        WorldConfig config;

    public:
        WorldBase() {};

        /**
         * Returns the stored config.
         */
        WorldConfig get_config() { return config; };

        /**
        * First applies the new configuration, then calls `reset()`.
        */
        void reset(WorldConfig config);

        /**********************************************************************/
        /** Virtual functions *************************************************/
        /**********************************************************************/

        /**
        * Resets the world using the stored configuration.
        */
        virtual void reset() {};

        /**
         * Converts this world into a ros message.
         */
        virtual roboteam_msgs::World as_message() { return roboteam_msgs::World(); };

        /**
         * To be called when a detectionframe message is received.
         */
        virtual void detection_callback(const roboteam_msgs::DetectionFrame msg) {};

    };

}
