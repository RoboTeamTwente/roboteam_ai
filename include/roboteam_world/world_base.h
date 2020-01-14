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
    virtual proto::World as_message() const { return proto::World(); };

};

}
