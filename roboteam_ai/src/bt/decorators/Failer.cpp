#include "Failer.hpp"

namespace bt {

Node::Status Failer::Update() {
    Node::append_status("[Failer: executing child of type %s]", child->node_name().c_str());
    child->Tick();
    return Status::Failure;
}

std::string Failer::node_name() {
    return "Failer";
}

} // bt
