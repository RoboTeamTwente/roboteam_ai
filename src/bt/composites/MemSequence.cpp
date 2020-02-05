/*
 *   The MemSequence composite ticks each child node in order, and remembers what child it previously tried to tick.
 *   If a child fails or runs, the sequence returns the same status.
 *   In the next tick, it will try to run each child in order again.
 *   If all children succeeds, only then does the sequence succeed.
 */

#include "bt/composites/MemSequence.h"

namespace bt {
MemSequence::MemSequence() : Composite() {}

MemSequence::MemSequence(nvector children) : Composite(children) {}

void MemSequence::initialize() { index = 0; }

bt::Node::Status MemSequence::update() {
    if (HasNoChildren()) {
        return Status::Success;
    }

    // Keep going until a child behavior says it's running.
    while (index < children.size()) {
        auto &child = children.at(index);

        auto status = child->tick(world, field);

        // If the child fails, or keeps running, do the same.
        if (status != Status::Success) {
            return status;
        }

        index++;
    }

    return Status::Success;
}

void MemSequence::terminate(Node::Status s) { index = 0; }
}  // namespace bt
// bt
