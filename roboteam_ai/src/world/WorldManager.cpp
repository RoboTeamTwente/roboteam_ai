//
// Created by thijs on 21-3-19.
//

#include "WorldManager.h"

namespace rtt {
namespace ai {
namespace world {

void WorldManager::setup() {
    IOManager = new io::IOManager(true);
}

void WorldManager::loop() {
    while (ros::ok()) {
        updateROSData();

        updateField();
        updateWorld();
        updateGameAnalyzer();

    }
}

void WorldManager::updateROSData() {
    // make ROS world_state and geometry data globally accessible
    worldMsg = IOManager->getWorldState();
    geometryMsg = IOManager->getGeometryData();
    refereeMsg = IOManager->getRefereeData();
}

void WorldManager::updateWorld() {
    world->set_world(worldMsg);
}

void WorldManager::updateField() {
    field->set_field(geometryMsg.field);
}

void WorldManager::updateGameAnalyzer() {

}

}
}
}