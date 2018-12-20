/*
 *    The Selector composite ticks each child node in order.
 *    If a child succeeds or runs, the sequence returns the same status.
 *    In the next tick, it will try to run each child in order again.
 *    If all children fails, only then does the selector fail.
 */

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

} // bt
