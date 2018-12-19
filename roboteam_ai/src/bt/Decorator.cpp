#include "Decorator.hpp"

namespace bt {

void Decorator::addChild(Node::Ptr child) {
    this->child = child;
}

bool Decorator::HasNoChild() const {
    return child == nullptr;
}

void Decorator::terminate(Status s) {
    if (child->getStatus() == Status::Running) {
        child->terminate(child->getStatus());
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}

std::vector<Node::Ptr> Decorator::getChildren() {
    return std::vector<Node::Ptr>{child};
}
} // bt


