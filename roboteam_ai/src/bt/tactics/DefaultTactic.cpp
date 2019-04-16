#include <roboteam_ai/src/world/WorldData.h>
#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>
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
        status = Status::Running;
        return status;
    }

    for (unsigned long i = 0; i < amountToTick; i ++) {
        children.at(i)->tick();
    }

    return status == Status::Success ? status : Status::Running;

}

bt::DefaultTactic::DefaultTactic(std::string name, bt::Blackboard::Ptr blackboard,
        const std::vector<std::pair<std::string, RobotType>> &robots_) {

    convert(robots_);
    globalBB = std::move(blackboard);
    this->name = std::move(name);
}



void bt::DefaultTactic::claimRobots(int amount) {

    // for the amount of robots we still need
    for (int i = 0; i < amount; i++) {
        auto toClaim = getNextClaim();

//        std::cout << toClaim.first << std::endl;

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

        // we need to store the robot id for the robot which we will remove
        // because after releaseRobotforRole, findRobotforRole will return -1 instead
        int robotToReleaseId = dealer::findRobotForRole(getLastClaim().first);
        dealer::releaseRobotForRole(getLastClaim().first); // free it in robotdealer
        robotIDs.erase(robotToReleaseId); // erase it in our array as well
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

    rtt::ai::analysis::AnalysisReport report = * rtt::ai::analysis::GameAnalyzer::getInstance().getMostRecentReport();
    rtt::ai::analysis::BallPossession possession = report.ballPossession;
    rtt::ai::analysis::PlayStyle style = maker.getRecommendedPlayStyle(possession);

    if (thisType == Defensive) {
        amountToTick = style.amountOfDefenders;
    }
    else if (thisType == Middle) {
        amountToTick = style.amountOfMidfielders;
    }
    else if (thisType == Offensive) {
        amountToTick = style.amountOfAttackers;
    }
    else if (rtt::ai::robotDealer::RobotDealer::usesSeparateKeeper()) {
        amountToTick = rtt::ai::world::world->getUs().size() - 1;
    } else {
        amountToTick = rtt::ai::world::world->getUs().size();
    }
}
void bt::DefaultTactic::convert(const std::vector<std::pair<std::string, RobotType>> &unit) {
    int counter = 1;
    for (const auto &robot : unit) {
        std::tuple<int, std::string, RobotType> temp = std::tuple<int, std::string, RobotType>(counter, robot.first, robot.second);
        robots.push_back(temp);
        counter ++;
    }
}



