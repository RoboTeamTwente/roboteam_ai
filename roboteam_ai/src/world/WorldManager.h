//
// Created by thijs on 21-3-19.
//

#ifndef ROBOTEAM_AI_WORLDMANAGER_H
#define ROBOTEAM_AI_WORLDMANAGER_H

#include "../io/IOManager.h"
#include "../treeinterp/BTFactory.h"
#include "ros/ros.h"
#include "World.h"
#include "Field.h"
#include "FutureWorld.h"
#include "WorldData.h"

namespace rtt {
namespace ai {
namespace world {

class WorldManager {
    private:
        io::IOManager* IOManager;

    public:
        void setup();
        void loop();
};

}
}
}

#endif //ROBOTEAM_AI_WORLDMANAGER_H
