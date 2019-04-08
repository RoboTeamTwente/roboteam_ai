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
    children.push_back(newChild);
}

void Tactic::terminate(Node::Status s) {
    rtt::ai::robotDealer::RobotDealer::removeTactic(name);
    for (const auto &child : children) {
        child->terminate(child->getStatus());
    }
    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
    claimedRobots = 0;
}

void Tactic::askForRobots() {

}
Node::Status Tactic::update() {
 // should always be overrriden
    return Status::Failure;
}

std::string Tactic::node_name() {
    return name;
}

std::vector<Node::Ptr> Tactic::getChildren() {
    return children;
}

}