/*
 *    The Selector composite ticks each child node in order.
 *    If a child succeeds or runs, the sequence returns the same status.
 *    In the next tick, it will try to run each child in order again.
 *    If all children fails, only then does the selector fail.
 */

#include "bt/composites/Selector.hpp"

namespace bt {

    /**
     * Use this constructor when you want to initialize the children of the selector using a vector.
     * The children are added sequentially, so the first element in the array will be the leftmost child of the selector
     * @param children vector of nodes that will be the children of this selector node
     */
    Selector::Selector(std::vector<std::shared_ptr<bt::Node>> children) {
        for (int i = 0; i < children.size(); i++) {
            this->addChild(children[i]);
        }
    }

Node::Status Selector::update() {
    // Keep going until a child behavior says it's running.
    for (auto &child : children) {
        auto status = child->tick(world, field);

        // If the child succeeds, or keeps running, do the same.
        if (status != Status::Failure) {
            return status;
        }
    }

    return Status::Failure;
}

    Selector::Selector() {

    }

} // bt
