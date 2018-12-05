#include "Inverter.hpp"

namespace bt {

Node::Status Inverter::update() {
    Node::append_status("[Inverter: executing child of type %s]", child->node_name().c_str());
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
