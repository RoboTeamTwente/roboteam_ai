
//
// Created by baris on 29-11-18.
//

#include <roboteam_ai/src/world/WorldData.h>
#include <roboteam_ai/src/world/World.h>
#include "DefaultTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = rtt::ai::robotDealer::RobotDealer;

void bt::DefaultTactic::initialize() {
    parseType(properties->getString("TacticType"));

    updateStyle();
    robotsNeeded = amountToTick;
    updateRobots();
}

bt::Node::Status bt::DefaultTactic::update() {
    updateStyle();

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
        const std::map<std::string, RobotType> &robots_) {

    convert(robots_);
    globalBB = std::move(blackboard);
    this->name = std::move(name);
    robotsNeeded = static_cast<int>(robots.size());
}



void bt::DefaultTactic::claimRobots() {
    for (int i = 0; i < robotsNeeded; i ++) {
        auto toClaim = getNextClaim();


        robotIDs.insert(dealer::claimRobotForTactic(toClaim.second, name, toClaim.first));
        if (robotIDs.find(- 1) == robotIDs.end()) {
            claimedRobots++;
            claimIndex++;
        }
        else robotIDs.erase(- 1);

    }
}

bool bt::DefaultTactic::updateRobots() {

    robotsNeeded = amountToTick - claimedRobots;

    if (robotsNeeded < 0) {
        disClaimRobots();
        return true;
    }
    else if (robotsNeeded > 0) {
        claimRobots();
        return (claimedRobots == amountToTick);

    }
    return true;

}
void bt::DefaultTactic::disClaimRobots() {
    int amount = robotsNeeded*- 1;
    for (int i = 0; i < amount; i ++) {
        dealer::releaseRobotForRole(getLastClaim().first);
        claimedRobots --;
        claimIndex --;
    }

}

std::pair<std::string, bt::Tactic::RobotType> bt::DefaultTactic::getNextClaim() {
    for (auto robot : robots) {
        if (std::get<0>(robot) == (claimIndex + 1)) {
            // claimIndex ++;
            return {std::get<1>(robot), std::get<2>(robot)};
        }
    }
}
std::pair<std::string, bt::Tactic::RobotType> bt::DefaultTactic::getLastClaim() {
    for (auto robot : robots) {
        if (std::get<0>(robot) == (claimIndex)) {
            return {std::get<1>(robot), std::get<2>(robot)};
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
    else {
        thisType = General;
    }

}
void bt::DefaultTactic::updateStyle() {
    rtt::ai::analysis::PlayStyle style = maker.getRecommendedPlayStyle();

    if (thisType == Defensive) {
        amountToTick = style.amountOfDefenders;
    }
    else if (thisType == Middle) {
        amountToTick = style.amountOfMidfielders;
    }
    else if (thisType == Offensive) {
        amountToTick = style.amountOfAttackers;
    }
    else {
        // All robots minus the keeper maybe
        amountToTick = rtt::ai::world::world->getUs().size() - 1;
        // amountToTick = 7;
    }
    std::cout << node_name() << " --- " << amountToTick << std::endl;

}
void bt::DefaultTactic::convert(
        const std::map<std::string, bt::Tactic::RobotType> &unit) {

    int counter = 1; // indexses start at 1 because fuck you
    for (const auto &robot : unit) {
        std::tuple<int, std::string, RobotType> temp = {counter, robot.first, robot.second};
        robots.push_back(temp);
        counter ++;
    }
}



