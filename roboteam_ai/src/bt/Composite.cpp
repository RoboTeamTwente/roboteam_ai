#include "Composite.hpp"

namespace bt {

void Composite::addChild(Node::Ptr child) {
    children.push_back(child);
}

bool Composite::HasNoChildren() const {
    return children.empty();
}

void Composite::terminate(Status s) {
    for (auto child : children) {
        if (child->getStatus() == Status::Running) {
            child->terminate(child->getStatus());
        }
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}

std::vector<Node::Ptr> Composite::getChildren() {
    return children;
}

} // bt
