/*
 * The Repeater decorator repeats infinitely or to a limit until the child returns success.
 */

#include "bt/decorators/Repeater.hpp"

namespace bt {

void Repeater::initialize() {
    counter = 0;
    /**
     * You could have blackboard have some sort of member function
     * value_or() aka std::optional
     * 
     * properties->hasInt("limit").value_or(0);
     * 
     * properties->value_or<int>("limit", 0);
     */
    if (properties->hasInt("limit")) limit = properties->getInt("limit");
    else limit = 0;
}

Node::Status Repeater::update() {
    while (limit >= 0 && (limit != counter++ || limit == 0)) {
        child->tick(world, field);
        return Status::Running;
    }
    return Status::Success;
}

} // bt
