//
// Created by mrlukasbos on 24-1-19.
//

#include "AvoidBallTactic.h"
#include "../../utilities/RobotDealer.h"


bt::AvoidBallTactic::AvoidBallTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    this->name = std::move(name);
    globalBB = std::move(blackboard);
}

void bt::AvoidBallTactic::initialize() {
    std::vector<std::string> roleNames = {"avoid1", "avoid2", "avoid3", "avoid4", "avoid5", "avoid6", "avoid7", "avoid8"};

    // get the amount of robots to claim
    while (!rtt::ai::robotDealer::RobotDealer::getAvailableRobots().empty()) {
        robotIDs.insert(rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(
                rtt::ai::robotDealer::RobotType::RANDOM, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}



