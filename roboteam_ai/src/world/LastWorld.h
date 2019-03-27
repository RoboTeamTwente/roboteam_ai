//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_LASTWORLD_H
#define ROBOTEAM_AI_LASTWORLD_H

#include "WorldData.h"

namespace rtt {
namespace ai {
namespace world {



class LastWorld {
    private:
        using WorldDataPtr = std::shared_ptr<WorldData>;

        WorldBuffer worldBuffer;
    public:
        void addWorld(WorldData worldData);
        void addWorld(WorldDataPtr &worldDataPtr);
        const WorldData &getPreviousWorld(unsigned int worldsBack = 1);
};

extern LastWorld lastWorldObj;
extern LastWorld* lastWorld;

}
}
}

#endif //ROBOTEAM_AI_LASTWORLD_H
