/*
 * The Failer decorator returns failure, regardless of what happens to the child.
 */

#include "include/roboteam_ai/bt/decorators/Failer.hpp"

namespace bt {

Node::Status Failer::update() {
    child->tick();
    return Status::Failure;
}

} // bt
