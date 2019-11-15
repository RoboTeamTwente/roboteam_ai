/*
 * The UntilSuccess decorator repeats until the child returns success and then returns success.
 */

#include "bt/decorators/UntilSuccess.hpp"

namespace bt {

Node::Status UntilSuccess::update() {
    // switches are about 25% faster, ish

    /**
     * https://stackoverflow.com/questions/97987/advantage-of-switch-over-if-else-statement
     */

    switch (child->tick(world, field)) {
        case Status::Success:
            return Status::Success;
        case Status::Waiting:
            return Status::Failure;
        default:
            return Status::Running;
    }

    // auto status = child->tick(world, field);

    // if (status == Status::Success) {
    //     return Status::Success;
    // }
    // else if (status == Status::Waiting) {
    //     return Status::Failure;
    // }
    // else {
    //     return Status::Running;
    // }
}

} // bt
