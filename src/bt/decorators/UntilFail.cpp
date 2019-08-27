/*
 *  The UntilFail decorator repeats until the child returns fail and then returns success.
 */

#include "include/roboteam_ai/bt/decorators/UntilFail.hpp"

namespace bt {

Node::Status UntilFail::update() {
    auto status = child->tick();

    if (status == Status::Failure) {
        return Status::Success;
    } else if (status == Status::Waiting) {
        return Status::Failure;
    } else {
        return Status::Running;
    }
}

} // bt
