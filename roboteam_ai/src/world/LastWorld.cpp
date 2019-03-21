//
// Created by thijs on 19-3-19.
//

#include "LastWorld.h"


namespace rtt {
namespace ai {
namespace world {

void LastWorld::updateLastWorld() {
    worldBuffer.addWorld(world);
    updatePredictedWorld();
}
const WorldData &LastWorld::getLastWorld(unsigned int w) {
    return worldBuffer.getLastWorld(w);
}

}
}
}