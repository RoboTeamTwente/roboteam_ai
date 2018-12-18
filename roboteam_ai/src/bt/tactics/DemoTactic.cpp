
//
// Created by baris on 05/11/18.
//

#include <utility>
#include "DemoTactic.h"


namespace bt {

DemoTactic::DemoTactic(std::string name, Blackboard::Ptr blackboard) {

    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void DemoTactic::setName(std::string newName) {
    name = std::move(newName);
}

void DemoTactic::initialize() {
    std::vector<std::string> roleNames = {"testRole"};

    while (claimedRobots < static_cast<int>(roleNames.size())) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, "DemoTactic", roleNames[claimedRobots]));
        if (robotIDs.find(- 1) == robotIDs.end()) {
            claimedRobots ++;
        } else {
            robotIDs.erase(- 1);
        }
    }
}

Node::Status DemoTactic::update() {
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

void DemoTactic::terminate(Status s) {
    dealer::removeTactic("DemoTactic");
    if (child->getStatus() == Status::Running) {
        child->terminate(child->getStatus());
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}

std::string DemoTactic::node_name() {
    return "DemoTactic";
}


} // bt










