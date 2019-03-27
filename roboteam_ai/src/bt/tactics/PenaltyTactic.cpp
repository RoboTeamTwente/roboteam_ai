//
// Created by baris on 18-3-19.
//

#include "PenaltyTactic.h"
namespace bt {

PenaltyTactic::PenaltyTactic(std::string name, Blackboard::Ptr blackboard) {

}
void PenaltyTactic::initialize() {
    std::vector<std::string> roleNames = {"formation1", "formation2", "formation3", "formation4", "formation5", "formation6", "formation7"};

    // get the amount of robots to claim
    while (!dealer::getAvailableRobots().empty()) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::RANDOM, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}
}