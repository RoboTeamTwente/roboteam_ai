/*
 *   The Succeeder decorator returns success, regardless of what happens to the child.
 */

#include "bt/decorators/Succeeder.h"

namespace bt {

Node::Status Succeeder::update() {
    child->tick(world, field);
    return Status::Success;
}

}  // namespace bt
