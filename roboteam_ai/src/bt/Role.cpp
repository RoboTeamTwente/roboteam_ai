#include <utility>

//
// Created by baris on 07/11/18.
//

#include "Role.h"

namespace bt {

void Role::initialize() {
    // Get the robot ID for this Role

}
Node::Status Role::update() {
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
void Role::addChild(Node::Ptr newChild) {
    this->child = newChild;

}
std::string Role::node_name() {
    return name;

}
Role::Role(std::string name) {
    this->name = std::move(name);
    globalBB = std::make_shared<Blackboard>();

}

std::vector<Node::Ptr> Role::getChildren() {
    if (child) {
        return std::vector<Node::Ptr>{child};
    }
    return {};
}



void Role::terminate(Status s) {
    for (auto child : getChildren()) {
        // if (child->getStatus() == Status::Running) {
        child->terminate(child->getStatus());
        //   }
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}

}