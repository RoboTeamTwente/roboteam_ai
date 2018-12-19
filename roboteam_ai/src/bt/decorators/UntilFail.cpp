#include "UntilFail.hpp"

namespace bt {

Node::Status UntilFail::update() {
    auto status = child->tick();

    if (status == Status::Failure) {
        return Status::Success;
    }
    else if (status == Status::Waiting) {
        return Status::Failure;
    }
    else /* if (status == Status::Running || status == Status::Success) */ {
        // If the status was anything else, we just keep running
        return Status::Running;
    }
}

std::string UntilFail::node_name() {
    return "UntilFail";
}

} // bt
