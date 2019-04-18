//
// Created by thijs on 21-3-19.
//

#include "WorldManager.h"
#include <roboteam_ai/src/utilities/StrategyManager.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>

namespace rtt {
namespace ai {
namespace world {

void WorldManager::setup() {
    IOManager = new io::IOManager(true, false);
}

void WorldManager::loop() {
    while (ros::ok()) {

        unsigned char changes = updateROSData();
        if (changes == 0b0000) continue;
        
        if (changes & 0b0001) updateReferee();
        if (changes & 0b0010) updateWorld();
        if (changes & 0b0100) updateGeometry();
        if (changes & 0b1000) updateRobotFeedback();
    }
}

unsigned char WorldManager::updateROSData() {
    // make ROS world_state and geometry data globally accessible
    unsigned char anyMsgHasChanged = 0b000;

    auto oldWorldMsg = worldMsg;
    auto oldGeometryMsg = geometryMsg;
    auto oldRefereeMsg = refereeMsg;
    auto oldRobotFeedbackMsg = robotFeedbackMsg;

    refereeMsg = IOManager->getRefereeData();
    worldMsg = IOManager->getWorldState();
    geometryMsg = IOManager->getGeometryData();
    robotFeedbackMsg = IOManager->getRobotFeedback();

    // refereeMsg:  0b001
    anyMsgHasChanged = anyMsgHasChanged | refereeMsgChanged(oldRefereeMsg, refereeMsg);
    // worldMsg:    0b010
    anyMsgHasChanged = anyMsgHasChanged | worldMsgChanged(oldWorldMsg, worldMsg);
    // geometryMsg: 0b100
    anyMsgHasChanged = anyMsgHasChanged | geometryMsgChanged(worldMsg);

    return anyMsgHasChanged;
}

void WorldManager::updateReferee() {
    if (ai::interface::InterfaceValues::usesRefereeCommands()) {

        ai::StrategyManager strategyManager;
        // Warning, this means that the names in strategy manager needs to match one on one with the JSON names
        // might want to build something that verifies this
        auto oldStrategy = BTFactory::getCurrentTree();
        std::string strategyName = strategyManager.getCurrentStrategyName(refereeMsg.command);
        if (oldStrategy != strategyName) {
            BTFactory::makeTrees();
            BTFactory::setCurrentTree(strategyName);
        }


        auto oldKeeperTree = BTFactory::getKeeperTreeName();
        std::string keeperTreeName = strategyManager.getCurrentKeeperTreeName(refereeMsg.command);
        if (oldKeeperTree != keeperTreeName) {
            std::cout << oldKeeperTree <<  "vs " << keeperTreeName << std::endl;
            BTFactory::makeTrees();
            BTFactory::setKeeperTree(keeperTreeName);
        }


        // if there is a referee, we always want to have a separate keeper tree.
        robotDealer::RobotDealer::setUseSeparateKeeper(true);
//        robotDealer::RobotDealer::setKeeperID(refereeMsg.us.goalie);
    }
}

void WorldManager::updateWorld() {
    world->updateWorld(worldMsg);
}

void WorldManager::updateGeometry() {
    world::field->set_field(geometryMsg.field);
}

void WorldManager::updateGameAnalyzer(const WorldData &worldData) {
    //TODO:
}


unsigned char WorldManager::refereeMsgChanged(roboteam_msgs::RefereeData oldR, roboteam_msgs::RefereeData newR) {
    unsigned char bit = 0b000;

    bool msgChanged = true;
    msgChanged |= oldR.packet_timestamp != newR.packet_timestamp;
    msgChanged |= oldR.command_timestamp != newR.command_timestamp;

    if (msgChanged) bit = 0b001;

    return bit;
}

unsigned char WorldManager::worldMsgChanged(roboteam_msgs::World oldW, roboteam_msgs::World newW) {
    unsigned char bit = 0b000;

    bool msgChanged = false;
    msgChanged |= oldW.time != newW.time;
    msgChanged |= (Vector2)oldW.ball.pos != newW.ball.pos;
    msgChanged |= (Vector2)oldW.ball.vel != newW.ball.vel;

    if (msgChanged) bit = 0b010;

    return bit;
}

unsigned char WorldManager::geometryMsgChanged(roboteam_msgs::World newG) {
    unsigned char bit = 0b000;

    double updateEveryThisManySeconds = 1.0;
    static double lastUpdatedTime = 0.0;
    bool msgChanged = newG.time - lastUpdatedTime > updateEveryThisManySeconds;

    if (msgChanged) {
        lastUpdatedTime = newG.time;
        bit = 0b100;
    }

    return bit;
}

void WorldManager::updateRobotFeedback() {
// TODO
}

}
}
}