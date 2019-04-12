#include <roboteam_ai/src/world/WorldData.h>
#include <roboteam_ai/src/world/World.h>
#include "DefaultTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = rtt::ai::robotDealer::RobotDealer;

void bt::DefaultTactic::initialize() {

    // determine the tactic
    parseType(properties->getString("TacticType"));

    // determine the amount of robots we need
    // we store this in the amountToTick variable
    updateStyle();
    updateRobots();
}

bt::Node::Status bt::DefaultTactic::update() {
    updateStyle();

    if (!updateRobots()) {
        std::cout << "waiting...until more robots are free" << std::endl;
        status = Status::Running;
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
}



void bt::DefaultTactic::claimRobots(int amount) {

    // for the amount of robots we still need
    for (int i = 0; i < amount; i++) {
        auto toClaim = getNextClaim();
        robotIDs.insert(dealer::claimRobotForTactic(toClaim.second, toClaim.first, name));
        if (robotIDs.find(- 1) != robotIDs.end()) {
            robotIDs.erase(-1);
        }
    }
}

bool bt::DefaultTactic::updateRobots() {
    int robotsNeeded = amountToTick - robotIDs.size();

    // if we need a negative amount of robots we have too much, so we need to disclaim robots
    // if we have 0 robots needed it is perfect
    // if we have a positive amount of robots needed we must claim more
    if (robotsNeeded < 0) {
        disClaimRobots(-robotsNeeded);
    }
    else if (robotsNeeded > 0) {
        claimRobots(robotsNeeded);
    }
    return (robotIDs.size() == amountToTick);
}
void bt::DefaultTactic::disClaimRobots(int amount) {
    for (int i = 0; i < amount; i ++) {


        // ccareful: this order matters!!!

        // THIS IS THE FIX
        // storing the value of the robot that was last claimed.
        // findRobotForRole returns -1 when the robot is released first.
        // but if it is released later, then it is released from robotIDS which makes getLastClaim fail.

        auto thingie = dealer::findRobotForRole(getLastClaim().first);

        dealer::releaseRobotForRole(getLastClaim().first); // free it in robotdealer

        robotIDs.erase(thingie); // erase it in our array as well


    }
}

std::pair<std::string, bt::Tactic::RobotType> bt::DefaultTactic::getNextClaim() {
    for (auto robot : robots) {
        if (std::get<0>(robot) == (robotIDs.size() +1)) {
            return {std::get<1>(robot), std::get<2>(robot)};
        }
    }
}
std::pair<std::string, bt::Tactic::RobotType> bt::DefaultTactic::getLastClaim() {
    for (auto robot : robots) {
        if (std::get<0>(robot) == (robotIDs.size())) {
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
void bt::DefaultTactic::convert(const std::map<std::string, bt::Tactic::RobotType> &unit) {

    int counter = 1;
    for (const auto &robot : unit) {
        std::tuple<int, std::string, RobotType> temp = {counter, robot.first, robot.second};
        robots.push_back(temp);
        counter ++;
    }
}



