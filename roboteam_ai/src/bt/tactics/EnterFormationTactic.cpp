//
// Created by mrlukasbos on 23-1-19.
//

#include "EnterFormationTactic.h"
#include "../../utilities/RobotDealer.h"

using dealer = robotDealer::RobotDealer;

bt::Node::Status bt::EnterFormationTactic::update() {
    auto status = child->tick();

    if (status == Status::Success) {
        return Status::Success;
    }

    else /* if (status == Status::Failure || status == Status::Running) */ {
        // If the status was anything but success/invalid, keep running
        return Status::Running;
    }
}


bt::EnterFormationTactic::EnterFormationTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void bt::EnterFormationTactic::setName(std::string newName) {
    name = std::move(newName);
}

void bt::EnterFormationTactic::initialize() {
    std::vector<std::string> roleNames = {"formation1", "formation2", "formation3", "formation4", "formation5", "formation6", "formation7"};

    // get the amount of robots to claim
    while (!dealer::getAvailableRobots().empty()) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}

void bt::EnterFormationTactic::terminate(bt::Node::Status s) {
    dealer::removeTactic(name);
    child->terminate(child->getStatus());
    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
    claimedRobots = 0;
}

std::string bt::EnterFormationTactic::node_name() {
    return name;
}


