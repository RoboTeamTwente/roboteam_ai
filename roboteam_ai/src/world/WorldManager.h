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

namespace rtt {
namespace ai {
namespace world {

class WorldManager {
    private:
        io::IOManager* IOManager;
        roboteam_msgs::World worldMsg;
        roboteam_msgs::GeometryData geometryMsg;
        roboteam_msgs::RefereeData refereeMsg;

        void updateROSData();
        void updateWorld();
        void updateField();
        void updateGameAnalyzer();
    public:
        void setup();
        void loop();

};

}
}
}

#endif //ROBOTEAM_AI_WORLDMANAGER_H
