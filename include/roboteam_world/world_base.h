#pragma once

#include "roboteam_proto/World.pb.h"

namespace world {

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

};

}
