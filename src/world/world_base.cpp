#include "world_base.h"


namespace rtt {

    void WorldBase::reset(WorldConfig config) {
        this->config = config;
        reset();
    }

}
