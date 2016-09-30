#include "roboteam_world/world/world_base.h"


namespace rtt {

    void WorldBase::config_reset(WorldConfig config) {
        this->config = config;
        reset();
    }

}
