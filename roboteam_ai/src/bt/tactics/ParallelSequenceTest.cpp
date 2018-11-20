//
// Created by robzelluf on 11/13/18.
//

#include "ParallelSequenceTest.h"
#include "../../utilities/World.h"
#include <utility>

namespace bt {
ParallelSequenceTactic::ParallelSequenceTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void ParallelSequenceTactic::setName(std::string newName) {
    name = std::move(newName);
}

void ParallelSequenceTactic::Initialize() {

    std::vector<std::string> roleNames = {"role1", "role2", "role3", "role4", "role5"};
    while (!claimedRobots) {
        claimedRobots = true;
        for (auto &roleName : roleNames) {
            claimedRobots &= dealer::claimRobotForTactic(robot::random, roleName, "ParallelSequenceTactic");
        }
    }
}

Node::Status ParallelSequenceTactic::Update() {
    auto status = child->Tick();

    if (status == Status::Success) {
        return Status::Success;
    }
    else if (status == Status::Invalid) {
        return Status::Failure;
    }
    else /* if (status == Status::Failure || status == Status::Running) */ {
        // If the status was anything but success/invalid, keep running
        return Status::Running;
    }
}

std::string ParallelSequenceTactic::node_name() {
    return "Parallel sequence tactic";
}

} //bt
