//
// Created by thijs on 15-11-18.
//

#include "VictoryDanceTactic.h"
#include "../../utilities/RobotDealer.h"

namespace bt {

using dealer = robotDealer::RobotDealer;

VictoryDanceTactic::VictoryDanceTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    this->name = std::move(name);
    globalBB = std::move(blackboard);
}

void VictoryDanceTactic::initialize() {

    std::vector<std::string> roleNames = {"victor1"};
    while (claimedRobots < static_cast<int>(roleNames.size())) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}


} //bt
