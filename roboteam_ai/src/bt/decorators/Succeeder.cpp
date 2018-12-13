#include "Succeeder.hpp"

namespace bt {

Node::Status Succeeder::update() {
    child->tick();
    return Status::Success;
}

std::string Succeeder::node_name() {
    return "Succeeder";
}

} // bt
