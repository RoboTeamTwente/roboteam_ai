//
// Created by thijs on 19-3-19.
//

#include "LastWorld.h"

namespace rtt {
namespace ai {
namespace world {

LastWorld lastWorldObj;
LastWorld* lastWorld = &lastWorldObj;

void LastWorld::addWorld(const WorldData worldData) {
    worldBuffer.addNewWorld(worldData);
}

void LastWorld::addWorld(WorldDataPtr &worldDataPtr) {
    WorldData worldData = *worldDataPtr.get();
    addWorld(worldData);
}

const WorldData &LastWorld::getPreviousWorld(unsigned int worldsBack) {
    return worldBuffer.getPreviousWorld(worldsBack);
}

}
}
}