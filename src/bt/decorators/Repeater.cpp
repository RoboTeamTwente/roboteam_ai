/*
 * The Repeater decorator repeats infinitely or to a limit until the child returns success.
 */

#include "bt/decorators/Repeater.h"

namespace bt {

void Repeater::initialize() {
    counter = 0;
    if (properties->hasInt("limit"))
        limit = properties->getInt("limit");
    else
        limit = 0;
}

Node::Status Repeater::update() {
    while (limit >= 0 && (limit != counter++ || limit == 0)) {
        child->tick(world, field);
        return Status::Running;
    }
    return Status::Success;
}

}  // namespace bt
