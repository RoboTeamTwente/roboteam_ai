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
        virtual proto::World as_message() const { return proto::World(); };

        /**
         * To be called when a detectionframe message is received.
         */
        virtual void detection_callback(const proto::SSL_DetectionFrame msg) {};

    };

}
