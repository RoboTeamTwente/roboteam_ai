//
// Created by baris on 29-11-18.
//

#include "DefaultTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = robotDealer::RobotDealer;

bt::Node::Status bt::DefaultTactic::update() {
    if (!updateRobots()) {
        status = Status::Waiting;
        return status;
    }

    for (unsigned long i = 0; i < amountToTick; i++) {
        children.at(i)->tick();
    }

    return status == Status::Success ? status : Status::Running;

}

bt::DefaultTactic::DefaultTactic(std::string name, bt::Blackboard::Ptr blackboard,
        std::map<std::string, robotType> robots_) {

    robots = std::move(robots_);
    globalBB = std::move(blackboard);
    this->name = std::move(name);
    robotsNeeded = static_cast<int>(robots.size());
}

void bt::DefaultTactic::initialize() {
    updateRobots();
}

void bt::DefaultTactic::claimRobots() {

    // TODO claim in a smart way with the map

    for (const auto &role : robots) {
        robotIDs.insert(dealer::claimRobotForTactic(role.second, name, role.first));
        if (robotIDs.find(- 1) == robotIDs.end()) claimedRobots ++;
        else robotIDs.erase(- 1);
    }
}

void bt::DefaultTactic::setRoleAmount(int amount) {
    std::lock_guard<std::mutex> lock(amountMutex);
    previousAmount = amountToTick;
    amountToTick = amount;
}

bool bt::DefaultTactic::updateRobots() {
    {
        std::lock_guard<std::mutex> lock(amountMutex);
        robotsNeeded = amountToTick - claimedRobots;
    }
    if (robotsNeeded < 0) {
        disClaimRobots();
        return true;
    }
    else if (robotsNeeded > 0) {
        claimRobots();
        {
            std::lock_guard<std::mutex> lock(amountMutex);
            return (claimedRobots == amountToTick);
        }
    }
    return true;

}
void bt::DefaultTactic::disClaimRobots() {

}



