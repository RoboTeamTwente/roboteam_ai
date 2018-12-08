//
// Created by mrlukasbos on 6-12-18.
//

#include "HaltTactic.h"
#include <utility>

namespace bt {

Node::Status HaltTactic::update() {
    auto status = child->tick();

    if (status == Status::Success) {
        return Status::Success;
    }
    else if (status == Status::Waiting) {
        return Status::Failure;
    }
    else /* if (status == Status::Failure || status == Status::Running) */ {
        // If the status was anything but success/invalid, keep running
        return Status::Running;
    }
}

HaltTactic::HaltTactic(std::string name, Blackboard::Ptr blackboard ) {
    globalBB = std::move(blackboard);
    setName (std::move(name));
}

void HaltTactic::setName(std::string newName) {
    name = std::move(newName);
}

void HaltTactic::initialize() {
    std::vector<std::string> roleNames = {"1", "2", "3", "4",
                                          "5", "6", "7", "8"};
    while (claimedRobots < roleNames.size()) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) {
            claimedRobots++;
        } else {
            robotIDs.erase(-1);
        }
    }
}

void HaltTactic::terminate(bt::Node::Status s) {
    dealer::removeTactic(name);
    child->terminate(child->getStatus());

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
    claimedRobots = 0;
}

std::string HaltTactic::node_name() {
    return name;
}

} // bt



