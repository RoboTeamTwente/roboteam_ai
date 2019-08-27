/*
 *   The Succeeder decorator returns success, regardless of what happens to the child.
 */

#include "include/roboteam_ai/bt/decorators/Succeeder.hpp"

namespace bt {

Node::Status Succeeder::update() {
    child->tick();
    return Status::Success;
}

} // bt
