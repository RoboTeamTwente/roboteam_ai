#include "Succeeder.hpp"

namespace bt {

Node::Status Succeeder::update() {
    Node::append_status("[Succeeder: executing child of type %s]", child->node_name().c_str());
    child->tick();
    return Status::Success;
}

std::string Succeeder::node_name() {
    return "Succeeder";
}

} // bt
