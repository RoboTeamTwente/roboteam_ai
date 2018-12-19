//
// Created by thijs on 15-11-18.
//

#include "VictoryDanceTactic.h"

namespace bt {

VictoryDanceTactic::VictoryDanceTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void VictoryDanceTactic::setName(std::string newName) {
    name = std::move(newName);
}

void VictoryDanceTactic::initialize() {

    std::vector<std::string> roleNames = {"victor1"};
    while (claimedRobots < static_cast<int>(roleNames.size())) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, name, roleNames[claimedRobots]));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}

Node::Status VictoryDanceTactic::update() {
    auto status = child->tick();

    if (status == Status::Success) {
        return Status::Success;
    }

    else /* if (status == Status::Failure || status == Status::Running) */ {
        // If the status was anything but success/invalid, keep running
        return Status::Running;
    }
}

void VictoryDanceTactic::terminate(Status s) {

    dealer::removeTactic(name);

    child->terminate(child->getStatus());

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
    claimedRobots = 0;

}
std::string VictoryDanceTactic::node_name() {
    return "Parallel sequence tactic";
}

} //bt
