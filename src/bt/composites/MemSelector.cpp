/*
 *    The Selector composite ticks each child node in order, and remembers what child it previously tried to tick.
 *    If a child succeeds or runs, the sequence returns the same status.
 *    In the next tick, it will try to run each child in order again.
 *    If all children fails, only then does the selector fail.
 */

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

} //bt
