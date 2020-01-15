//
// Created by thijs on 19-3-19.
//

#include "world/History.h"

namespace rtt::ai::world {

void History::addWorld(const WorldData &worldData) {
    worldBuffer.addNewWorld(worldData);
}

void History::addWorld(WorldDataPtr &worldDataPtr) {
    WorldData worldData = WorldData(worldDataPtr);
    addWorld(worldData);
}

const WorldData History::getPreviousWorld(int worldsBack) {
    return worldBuffer.getPreviousWorld(worldsBack);
}

}