//
// Created by mrlukasbos on 23-1-19.
//

#include "EnterFormationTactic.h"



bt::EnterFormationTactic::EnterFormationTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    this->name = std::move(name);
    globalBB = std::move(blackboard);
}

void bt::EnterFormationTactic::initialize() {
    std::vector<std::string> roleNames = {"formation1", "formation2", "formation3", "formation4", "formation5", "formation6", "formation7"};

    // get the amount of robots to claim
    while (!rtt::ai::robotDealer::RobotDealer::getAvailableRobots().empty()) {
        robotIDs.insert(rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(
                rtt::ai::robotDealer::RobotType::RANDOM, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}



