//
// Created by baris on 06/11/18.
//

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

    if (child->getStatus() == Status::Running) {
        child->terminate(child->getStatus());
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}
void Tactic::askForRobots() {

}
Node::Status Tactic::update() {
    //Should always be overwritten
    return Status::Waiting;
}

std::string Tactic::node_name() {
    return name;
}

std::vector<Node::Ptr> Tactic::getChildren() {
    return std::vector<Node::Ptr>{child};
}

}