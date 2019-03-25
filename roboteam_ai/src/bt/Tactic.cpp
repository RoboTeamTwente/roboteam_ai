//
// Created by baris on 06/11/18.
//

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "Tactic.h"

namespace bt {

void bt::Tactic::initialize() {
    //Should always be overwritten
    askForRobots();
}

void Tactic::addChild(Node::Ptr newChild) {
    this->child = newChild;
}

void Tactic::terminate(Node::Status s) {
    rtt::ai::robotDealer::robotDealer->removeTactic(name);
    child->terminate(child->getStatus());
    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
    claimedRobots = 0;
}

void Tactic::askForRobots() {

}
Node::Status Tactic::update() {
    auto status = child->tick();

    if (status == Status::Success) {
        return Status::Success;
    }

    else /* if (status == Status::Failure || status == Status::Running) */ {
        // If the status was anything but success/invalid, keep running
        return Status::Running;
    }
}

std::string Tactic::node_name() {
    return name;
}

std::vector<Node::Ptr> Tactic::getChildren() {
    return std::vector<Node::Ptr>{child};
}

}