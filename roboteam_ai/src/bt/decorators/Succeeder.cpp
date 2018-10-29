#include "Succeeder.hpp"

namespace bt {

    Node::Status Succeeder::Update() {
        Node::append_status("[Succeeder: executing child of type %s]", child->node_name().c_str());
        child->Tick();
        return Status::Success;
    }

    std::string Succeeder::node_name() {
        return "Succeeder";
    }

} // bt
