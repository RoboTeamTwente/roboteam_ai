/*
 *   The Inverter decorator inverts the child node's status, i.e. failure becomes success and success becomes failure.
 *   If the child runs, the Inverter returns the status that it is running too.
 */

#include "bt/decorators/Inverter.h"

namespace bt {

Node::Status Inverter::update() {
    auto s = child->tick(world, field);

    if (s == Status::Success) {
        return Status::Failure;
    } else if (s == Status::Failure) {
        return Status::Success;
    }

    return s;
}

}  // namespace bt
