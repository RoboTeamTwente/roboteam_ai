/*
 * The Failer decorator returns failure, regardless of what happens to the child.
 */

#include "bt/decorators/Failer.h"

namespace bt {

Node::Status Failer::update() {
    child->tick(world, field);
    return Status::Failure;
}

}  // namespace bt
