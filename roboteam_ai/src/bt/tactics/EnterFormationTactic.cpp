//
// Created by mrlukasbos on 23-1-19.
//

#include "EnterFormationTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = robotDealer::RobotDealer;

bt::EnterFormationTactic::EnterFormationTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    this->name = std::move(name);
    globalBB = std::move(blackboard);
}

void bt::EnterFormationTactic::initialize() {
    std::vector<std::string> roleNames = {"formation1", "formation2", "formation3", "formation4", "formation5", "formation6", "formation7", "formation8", "formation9", "formation10"};

    // get the amount of robots to claim
    while (!dealer::getAvailableRobots().empty()) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}



