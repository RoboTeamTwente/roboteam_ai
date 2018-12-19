#include "Failer.hpp"

namespace bt {

Node::Status Failer::update() {
    child->tick();
    return Status::Failure;
}

std::string Failer::node_name() {
    return "Failer";
}

} // bt
