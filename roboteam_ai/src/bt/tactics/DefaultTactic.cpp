//
// Created by baris on 29-11-18.
//

#include "DefaultTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = robotDealer::RobotDealer;

bt::Node::Status bt::DefaultTactic::update() {
    if (claimedRobots != robotsNeeded) {
        claimRobots();
        status = Status::Waiting;
    }
    else {
        auto status = child->tick();

        if (status == Status::Success) {
            return Status::Success;
        }

        else {
            return Status::Running;
        }
    }
    return status;
}


bt::DefaultTactic::DefaultTactic(std::string name, bt::Blackboard::Ptr blackboard,
        std::map<std::string, robotType> robots_) : name(name){

    robots = std::move(robots_);
    globalBB = std::move(blackboard);
    setName(std::move(name));
    robotsNeeded = static_cast<int>(robots.size());
}

void bt::DefaultTactic::initialize() {
    claimRobots();
}

void bt::DefaultTactic::claimRobots() {

    for (const auto &role : robots) {
        robotIDs.insert(dealer::claimRobotForTactic(role.second, name, role.first));
        if (robotIDs.find(- 1) == robotIDs.end()) claimedRobots ++;
        else robotIDs.erase(- 1);
    }
}



