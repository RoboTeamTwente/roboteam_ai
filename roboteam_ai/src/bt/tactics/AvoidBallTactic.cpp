//
// Created by mrlukasbos on 24-1-19.
//

#include "AvoidBallTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = robotDealer::RobotDealer;

bt::AvoidBallTactic::AvoidBallTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    this->name = std::move(name);
    globalBB = std::move(blackboard);
}

void bt::AvoidBallTactic::initialize() {
    std::vector<std::string> roleNames = {"avoid1", "avoid2", "avoid3", "avoid4", "avoid5", "avoid6", "avoid7", "avoid8", "avoid9", "avoid10", "avoid11"};

    // get the amount of robots to claim
    while (!dealer::getAvailableRobots().empty()) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}



