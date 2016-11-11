#pragma once

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/World.h"


namespace rtt {

    class WorldBase {

    public:
        WorldBase() {};

        /**********************************************************************/
        /** Virtual functions *************************************************/
        /**********************************************************************/

        virtual ~WorldBase() {};

        /**
        * Resets the world.
        */
        virtual void reset() {};

        /**
         * Converts this world into a ros message.
         */
        virtual roboteam_msgs::World as_message() const { return roboteam_msgs::World(); };

        /**
         * To be called when a detectionframe message is received.
         */
        virtual void detection_callback(const roboteam_msgs::DetectionFrame msg) {};

    };

}
