/*
 *   The Sequence composite ticks each child node in order.
 *   If a child fails or runs, the sequence returns the same status.
 *   In the next tick, it will try to run each child in order again.
 *   If all children succeeds, only then does the sequence succeed.
*/

#include "bt/composites/Sequence.hpp"

namespace bt {
    /**
 * Use this constructor when you want to initialize the children of the sequence using a vector.
 * The children are added sequentially, so the first element in the array will be the leftmost child of the sequence
 * @param children vector of nodes that will be the children of this sequence node
 */
    Sequence::Sequence(std::vector<std::shared_ptr<bt::Node>> children) {
        for (int i = 0; i < children.size(); i++) {
            this->addChild(children[i]);
        }
    }

Node::Status Sequence::update() {
    if (HasNoChildren()) {
        return Status::Success;
    }

    // Keep going until a child behavior says it's running.
    for (auto &child : children) {
        auto status = child->tick(world, field);

        // If the child fails, or keeps running, do the same.
        if (status != Status::Success) {
            return status;
        }
    }

    return Status::Success;
}

    Sequence::Sequence() {

    }

} // bt
