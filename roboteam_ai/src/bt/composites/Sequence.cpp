#include "Sequence.hpp"

namespace bt {

Node::Status Sequence::update() {
    if (HasNoChildren()) {
        return Status::Success;
    }

    // Keep going until a child behavior says it's running.
    for (auto &child : children) {
        auto status = child->tick();

        // If the child fails, or keeps running, do the same.
        if (status != Status::Success) {
            return status;
        }
    }

    return Status::Success;
}

std::string Sequence::node_name() {
    return "Sequence";
}

} // bt
