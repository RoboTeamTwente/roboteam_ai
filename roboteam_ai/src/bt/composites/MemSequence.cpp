#include <iostream>
#include "MemSequence.hpp"

namespace bt {

void MemSequence::initialize() {
    index = 0;
}

bt::Node::Status MemSequence::update() {
    if (HasNoChildren()) {
        return Status::Success;
    }

    // Keep going until a child behavior says it's running.
    while (index < children.size()) {
        auto &child = children.at(index);

        auto status = child->tick();

        // If the child fails, or keeps running, do the same.
        if (status != Status::Success) {
            return status;
        }

        index ++;
    }

    return Status::Success;
}

std::string MemSequence::node_name() {
    return "MemSequence";
}

} // bt
