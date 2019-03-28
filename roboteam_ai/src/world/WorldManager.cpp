//
// Created by thijs on 21-3-19.
//

#include "WorldManager.h"

namespace rtt {
namespace ai {
namespace world {

void WorldManager::setup() {
    IOManager = new io::IOManager(true, false);
}

void WorldManager::loop() {
    while (ros::ok()) {
        updateROSData();
        std::cout << worldMsg.time << std::endl;
        if (lastWorldTime != worldMsg.time) {
            updateField();
            updateWorld();
            updateGameAnalyzer();
        }
    }
}

void WorldManager::updateROSData() {
    // make ROS world_state and geometry data globally accessible
    worldMsg = IOManager->getWorldState();
    geometryMsg = IOManager->getGeometryData();
    refereeMsg = IOManager->getRefereeData();
}

void WorldManager::updateWorld() {

    world->setWorld(worldMsg);

    auto worldDataPtr = world->getWorld();
    lastWorld->addWorld(worldDataPtr);
    processedWorld->update(worldDataPtr);

    lastWorldTime = worldMsg.time;
}

void WorldManager::updateField() {
    world::field->set_field(geometryMsg.field);
}

void WorldManager::updateGameAnalyzer() {
    //TODO:
}

}
}
}