//
// Created by baris on 07/11/18.
//

#include "Role.h"

namespace bt{

void Role::Initialize() {
    // Get the robot ID for this Role

}
Node::Status Role::Update() {
    return Status::Invalid;
}
void Role::AddChild(Node::Ptr newChild) {
    this->child = newChild;

}

}