#include "MemParallelSequence.h"

namespace bt {

MemParallelSequence::MemParallelSequence() { }

void MemParallelSequence::initialize() {
    for (auto &child : children) {
        memory[child] = Status::Running;
    }
    totalSuccess = 0;
    totalFailure = 0;
    }

bt::Node::Status MemParallelSequence::update() {
    for (auto &child : children) {
        if (memory[child] == Status::Running) {
            auto status = child->tick();
            if (status == Status::Success) {
                memory[child] = Status::Success;
                totalSuccess++;
            }
            if (status == Status::Failure) {
                memory[child] = Status::Failure;
                totalFailure++;
            }
        }
    }

    if (totalSuccess == static_cast<int>(children.size())) {
        return Status::Success;
    }

    if (totalSuccess + totalFailure == static_cast<int>(children.size())) {
        return Status::Failure;
    }

    return Status::Running;
}

}
