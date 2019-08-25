/*
 *   The Sequence composite ticks each child node in order.
 *   If a child fails or runs, the sequence returns the same status.
 *   In the next tick, it will try to run each child in order again.
 *   If all children succeeds, only then does the sequence succeed.
*/

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

} // bt
