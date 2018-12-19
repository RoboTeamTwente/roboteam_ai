//
// Created by thijs on 29-11-18.
//

#include "RandomTactic.h"


namespace bt {

RandomTactic::RandomTactic(std::string name, Blackboard::Ptr blackboard) {

    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void RandomTactic::setName(std::string newName) {
    name = std::move(newName);
}

void RandomTactic::initialize() {
    std::vector<std::string> roleNames = {"random1","random2","random3","random4","random5","random6","random7","random8"};

    while (claimedRobots < static_cast<int>(roleNames.size())) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, name, roleNames[claimedRobots]));
        if (robotIDs.find(- 1) == robotIDs.end()) {
            claimedRobots ++;
        } else {
            robotIDs.erase(- 1);
        }
    }
}

Node::Status RandomTactic::update() {
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

void RandomTactic::terminate(Status s) {
    dealer::removeTactic(name);
    if (child->getStatus() == Status::Running) {
        child->terminate(child->getStatus());
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}

std::string RandomTactic::node_name() {
    return name;
}


} // bt










