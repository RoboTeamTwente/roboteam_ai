/*
 * The Failer decorator returns failure, regardless of what happens to the child.
 */

#include "Failer.hpp"

namespace bt {

Node::Status Failer::update() {
    child->tick();
    return Status::Failure;
}

} // bt
