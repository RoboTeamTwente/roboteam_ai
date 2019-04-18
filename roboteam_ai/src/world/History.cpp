//
// Created by thijs on 19-3-19.
//

#include "History.h"

namespace rtt {
namespace ai {
namespace world {

void History::addWorld(const WorldData &worldData) {
    worldBuffer.addNewWorld(worldData);
}

void History::addWorld(WorldDataPtr &worldDataPtr) {
    WorldData worldData = *worldDataPtr.get();
    addWorld(worldData);
}

const WorldData &History::getPreviousWorld(unsigned int worldsBack) {
    return worldBuffer.getPreviousWorld(worldsBack);
}

}
}
}