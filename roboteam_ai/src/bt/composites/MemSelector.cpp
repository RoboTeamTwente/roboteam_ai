#include "MemSelector.hpp"

namespace bt {
void MemSelector::initialize() {
    index = 0;
}

Node::Status MemSelector::update() {
    if (HasNoChildren()) {
        return Status::Success;
    }

    // Keep going until a child behavior says it's running.
    while (index < children.size()) {
        auto &child = children.at(index);
        auto status = child->tick();

        // If the child succeeds, or keeps running, do the same.
        if (status != Status::Failure) {
            return status;
        }

        index ++;
    }

    return Status::Failure;
}

using Ptr = std::shared_ptr<MemSelector>;

std::string MemSelector::node_name() {
    return "MemSelector";
}

} //bt
