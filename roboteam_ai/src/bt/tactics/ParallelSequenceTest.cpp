//
// Created by robzelluf on 11/13/18.
//

#include "ParallelSequenceTest.h"


namespace bt {
ParallelSequenceTactic::ParallelSequenceTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void ParallelSequenceTactic::setName(std::string newName) {
    name = std::move(newName);
}

void ParallelSequenceTactic::initialize() {

    std::vector<std::string> roleNames = {"role1", "role2", "role3", "role4", "role5"};
    while (claimedRobots < static_cast<int>(roleNames.size())) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, "ParallelSequenceTactic", roleNames[claimedRobots]));
        if (robotIDs.find(- 1) == robotIDs.end()) {
            claimedRobots ++;
        } else {
            robotIDs.erase(- 1);
        }
    }
}

Node::Status ParallelSequenceTactic::update() {
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

void ParallelSequenceTactic::terminate(Status s) {
    dealer::removeTactic("ParallelSequenceTactic");

    if (child->getStatus() == Status::Running) {
        child->terminate(child->getStatus());
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}


std::string ParallelSequenceTactic::node_name() {
    return "Parallel sequence tactic";
}

} //bt
