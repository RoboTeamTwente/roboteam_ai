#include <utility>

//
// Created by baris on 07/11/18.
//

#include "Role.h"

namespace bt {

void Role::Initialize() {
    // Get the robot ID for this Role

}
Node::Status Role::Update() {
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
void Role::AddChild(Node::Ptr newChild) {
    this->child = newChild;

}
std::string Role::node_name() {
    return name;

}
Role::Role(std::string name) {
    this->name = std::move(name);
    globalBB = std::make_shared<Blackboard>();

}
}