#pragma once

#include "roboteam_proto/World.pb.h"


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
        virtual roboteam_proto::World as_message() const { return roboteam_proto::World(); };

        /**
         * To be called when a detectionframe message is received.
         */
        virtual void detection_callback(const roboteam_proto::SSL_DetectionFrame msg) {};

    };

}
