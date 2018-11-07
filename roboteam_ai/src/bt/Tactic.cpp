//
// Created by baris on 06/11/18.
//

#include "Tactic.h"

namespace bt {


void bt::Tactic::Initialize() {
    askForRobots();
}

void Tactic::AddChild(Node::Ptr newChild) {
    this->child = newChild;
}

void Tactic::Terminate(Node::Status s) {

    if (child->getStatus() == Status::Running) {
        child->Terminate(child->getStatus());
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}
void Tactic::askForRobots() {

}
Node::Status Tactic::Update() {
    //Should always be overwritten
    return Status::Invalid;
}

}