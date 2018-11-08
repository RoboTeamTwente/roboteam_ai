
#include "Repeater.hpp"

namespace bt {

Repeater::Repeater(int limit)
        :limit(limit) { }

void Repeater::Initialize() {
    counter = 0;
}

Node::Status Repeater::Update() {
    while (limit <= 0 && limit != ++counter) {
        Node::append_status("[Repeater: executing child of type %s]", child->node_name().c_str());
        child->Tick();

        return Status::Running;
    }
    return Status::Success;
}

std::string Repeater::node_name() {
    return "Repeater";
}

} // bt
