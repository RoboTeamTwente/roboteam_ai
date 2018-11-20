
//
// Created by baris on 05/11/18.
//

#include <utility>
#include "DemoTactic.h"
#include "../../utilities/World.h"

namespace bt {

DemoTactic::DemoTactic(std::string name, Blackboard::Ptr blackboard) {

    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void DemoTactic::setName(std::string newName) {
    name = std::move(newName);
}

void DemoTactic::Initialize() {

    std::string roleName = "testRole";
    while (!claimedRobots) {
        claimedRobots = dealer::claimRobotForTactic(robot::random, roleName, "testTactic");
    }
}

Node::Status DemoTactic::Update() {
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

void DemoTactic::Terminate(Status s) {

}

std::string DemoTactic::node_name() {
    return "Demo Tactic";
}


} // bt










