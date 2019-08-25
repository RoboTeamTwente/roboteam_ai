/*
 *   The Succeeder decorator returns success, regardless of what happens to the child.
 */

#include "Succeeder.hpp"

namespace bt {

Node::Status Succeeder::update() {
    child->tick();
    return Status::Success;
}

} // bt
