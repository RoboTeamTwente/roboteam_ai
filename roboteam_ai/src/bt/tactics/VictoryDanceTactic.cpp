//
// Created by thijs on 15-11-18.
//

#include "VictoryDanceTactic.h"
#include "../../utilities/World.h"
#include <utility>

namespace bt {

VictoryDanceTactic::VictoryDanceTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void VictoryDanceTactic::setName(std::string newName) {
    name = std::move(newName);
}

void VictoryDanceTactic::initialize() {

    std::vector<std::string> roleNames = {"victory1"};
    while (claimedRobots < roleNames.size()) {
        robotIDs.insert(dealer::claimRobotForTactic(robot::random, roleNames[claimedRobots], "ParallelSequenceTactic"));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}

Node::Status VictoryDanceTactic::update() {
    auto status = child->tick();

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

std::string VictoryDanceTactic::node_name() {
    return "Parallel sequence tactic";
}

} //bt
