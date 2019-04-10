//
// Created by baris on 29-11-18.
//

#include <roboteam_ai/src/analysis/DecisionMaker.h>
#include "DefaultTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = rtt::ai::robotDealer::RobotDealer;

/// call 0638424067 if you need help

bt::Node::Status bt::DefaultTactic::update() {
    if (! updateRobots()) {
        status = Status::Waiting;
        return status;
    }

    for (unsigned long i = 0; i < amountToTick; i ++) {
        children.at(i)->tick();
    }

    return status == Status::Success ? status : Status::Running;

}

bt::DefaultTactic::DefaultTactic(std::string name, bt::Blackboard::Ptr blackboard,
        std::map<std::string, RobotType> robots_) {

    robots = std::move(robots_);
    globalBB = std::move(blackboard);
    this->name = std::move(name);
    robotsNeeded = static_cast<int>(robots.size());
}

void bt::DefaultTactic::initialize() {

    parseType(properties->getString("TacticType"));

    rtt::ai::analysis::DecisionMaker maker;
    rtt::ai::analysis::PlayStyle style = maker.getRecommendedPlayStyle();

    if (thisType == Defensive) {
        amountToTick = style.amountOfDefenders;
    } else if (thisType == Middle) {
        amountToTick = style.amountOfMidfielders;
    } else if (thisType == Offensive) {
        amountToTick = style.amountOfAttackers;
    } else {
        amountToTick = 7;
        // TODO: maybe look at the field and how many robots we can get instead of this 7.
    }
    robotsNeeded = amountToTick;
    updateRobots();
}

void bt::DefaultTactic::claimRobots() {
    for (int i = claimedRobots; i < robotsNeeded; i ++) {
        auto toClaim = getNextClaim();
        robotIDs.insert(dealer::claimRobotForTactic(toClaim.second, name, toClaim.first));
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
    dealer::releaseRobotForRole(getLastClaim().first);
    claimedRobots--;
    claimIndex --;

}

std::pair<std::string, bt::Tactic::RobotType> bt::DefaultTactic::getNextClaim() {
    int counter = 0;
    for (auto robot : robots) {
        if (counter == (claimIndex + 1)) {
            claimIndex ++;
            return robot;
        }
        else {
            counter ++;
        }
    }
}
std::pair<std::string, bt::Tactic::RobotType> bt::DefaultTactic::getLastClaim() {
    int counter = 0;
    for (auto robot : robots) {
        if (counter == (claimIndex)) {
            return robot;
        }
        else {
            counter ++;
        }
    }
}
void bt::DefaultTactic::parseType(std::string typee) {
    if (typee == "Offensive") {
        thisType = Offensive;
    }
    else if (typee == "Middle") {
        thisType = Middle;
    }
    else if (typee == "Defensive") {
        thisType = Defensive;
    }
    else{
        thisType = General;
    }

}



