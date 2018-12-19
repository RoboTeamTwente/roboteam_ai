#include "Inverter.hpp"

namespace bt {

Node::Status Inverter::update() {
    auto s = child->tick();

    if (s == Status::Success) {
        return Status::Failure;
    }
    else if (s == Status::Failure) {
        return Status::Success;
    }

    return s;
}

std::string Inverter::node_name() {
    return "Inverter";
}

}
