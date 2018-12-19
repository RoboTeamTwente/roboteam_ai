#include "Selector.hpp"

namespace bt {

Node::Status Selector::update() {
    // Keep going until a child behavior says it's running.
    for (auto &child : children) {
        auto status = child->tick();

        // If the child succeeds, or keeps running, do the same.
        if (status != Status::Failure) {
            return status;
        }
    }

    return Status::Failure;
}

std::string Selector::node_name() {
    return "Selector";
}

} // bt
