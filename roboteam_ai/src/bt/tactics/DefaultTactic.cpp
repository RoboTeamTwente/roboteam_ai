#include <roboteam_ai/src/world/WorldData.h>
#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>
#include "DefaultTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = rtt::ai::robotDealer::RobotDealer;

namespace bt {

void DefaultTactic::initialize() {
    // determine the tactic
    parseType(properties->getString("TacticType"));

    // determine the amount of robots we need
    // we store this in the amountToTick variable
    updateStyle();
    updateRobots();
}

Node::Status DefaultTactic::update() {
    updateStyle();

    if (!updateRobots()) {
        status = Status::Running;

        return status;
    }
    if (amountToTick > children.size()) {
        amountToTick = children.size();
    }

    for (int i = 0; i < amountToTick; i ++) {
        if (children.size() > i && children.at(i)) {
            children.at(i)->tick();
        } else {
            std::cerr << "trying to tick a non-existent robot!" << std::endl;
        }
    }

    return status == Status::Success ? status : Status::Running;

}

DefaultTactic::DefaultTactic(std::string name, Blackboard::Ptr blackboard,
                                 const std::vector<std::pair<std::string, RobotType>> &robots_) {

    convert(robots_);
    globalBB = std::move(blackboard);
    properties = globalBB;

    this->name = std::move(name);
}

void DefaultTactic::claimRobots(int amount) {

    // for the amount of robots we still need
    for (int i = 0; i < amount; i++) {
        auto toClaim = getNextClaim();
        robotIDs.insert(dealer::claimRobotForTactic(toClaim.second, toClaim.first, name));
        if (robotIDs.find(-1) != robotIDs.end()) {
            robotIDs.erase(-1);
        }
    }
}

bool DefaultTactic::updateRobots() {
    int robotsNeeded = amountToTick - robotIDs.size();

    // if we need a negative amount of robots we have too much, so we need to disclaim robots
    // if we have 0 robots needed it is perfect
    // if we have a positive amount of robots needed we must claim more
    if (robotsNeeded < 0) {
        disClaimRobots(-robotsNeeded);
    } else if (robotsNeeded > 0) {
        claimRobots(robotsNeeded);
    }
    return (static_cast<int>(robotIDs.size()) == amountToTick);
}

void DefaultTactic::disClaimRobots(int amount) {
    for (int i = 0; i < amount; i++) {

        // we need to store the robot id for the robot which we will remove
        // because after releaseRobotforRole, findRobotforRole will return -1 instead
        int robotToReleaseId = dealer::findRobotForRole(getLastClaim().first);
        dealer::releaseRobotForRole(getLastClaim().first); // free it in robotdealer
        robotIDs.erase(robotToReleaseId); // erase it in our array as well
    }
}

std::pair<std::string, rtt::ai::robotDealer::RobotType> DefaultTactic::getNextClaim() {
    for (auto &robot : robots) {
        if (std::get<0>(robot) == static_cast<int>(robotIDs.size() + 1)) {
            return std::make_pair(std::get<1>(robot), std::get<2>(robot));
        }
    }
    return {};
}

std::pair<std::string, rtt::ai::robotDealer::RobotType> DefaultTactic::getLastClaim() {
    for (auto &robot : robots) {
        if (std::get<0>(robot) == static_cast<int>(robotIDs.size())) {
            return std::make_pair(std::get<1>(robot), std::get<2>(robot));
        }
    }
    return {};
}

void DefaultTactic::parseType(const std::string &typee) {
    if (typee == "Offensive") {
        thisType = Offensive;
    } else if (typee == "Middle") {
        thisType = Middle;
    } else if (typee == "Defensive") {
        thisType = Defensive;
    } else {
        thisType = General;
    }

}

void DefaultTactic::updateStyle() {
    auto reportPtr = rtt::ai::analysis::GameAnalyzer::getInstance().getMostRecentReport();
    rtt::ai::analysis::PlayStyle style;
    if (reportPtr) {
        rtt::ai::analysis::AnalysisReport report = *reportPtr;
        rtt::ai::analysis::BallPossession possession = report.ballPossession;
        style = maker.getRecommendedPlayStyle(possession);
    } else {
        style = maker.getRecommendedPlayStyle(rtt::ai::analysis::BallPossession::NEUTRAL);
    }
    if (thisType == Defensive) {
        amountToTick = style.amountOfDefenders;
    } else if (thisType == Middle) {
        amountToTick = style.amountOfMidfielders;
    } else if (thisType == Offensive) {
        amountToTick = style.amountOfAttackers;
    }
    else if (rtt::ai::robotDealer::RobotDealer::usesSeparateKeeper() && rtt::ai::robotDealer::RobotDealer::keeperExistsInWorld()) {
        amountToTick = rtt::ai::world::world->getUs().size() - 1;
    } else {
        amountToTick = rtt::ai::world::world->getUs().size();
    }
}

void DefaultTactic::convert(const std::vector<std::pair<std::string, RobotType>> &unit) {
    int counter = 1;
    for (const auto &robot : unit) {
        std::tuple<int, std::string, RobotType> temp = std::tuple<int, std::string, RobotType>(counter, robot.first,
                                                                                               robot.second);
        robots.push_back(temp);
        counter++;
    }
}

void DefaultTactic::terminate(Node::Status s) {
    robotIDs = {};
    amountToTick = -1;
    rtt::ai::robotDealer::RobotDealer::removeTactic(name);
    for (const auto &child : children) {
        child->terminate(child->getStatus());
    }
    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
    claimedRobots = 0;
}

std::vector<Node::Ptr> DefaultTactic::getChildren() {
    return children;
}

void DefaultTactic::giveProperty(std::string a, std::string b) {
    properties->setString(a, b);
}

std::string DefaultTactic::node_name() {
    return this->name;
}


void DefaultTactic::addChild(Node::Ptr newChild) {
    children.push_back(newChild);
}

} // bt
