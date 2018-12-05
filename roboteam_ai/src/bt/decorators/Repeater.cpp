
#include "Repeater.hpp"

namespace bt {


void Repeater::initialize() {
    counter = 0;
    if (properties->hasInt("limit")) limit = properties->getInt("limit");
    else limit = 0;
}

Node::Status Repeater::update() {

    while (limit >= 0 && (limit != counter++ || limit == 0)) {
        Node::append_status("[Repeater: executing child of type %s]", child->node_name().c_str());
        child->tick();

        return Status::Running;
    }
    return Status::Success;
}

std::string Repeater::node_name() {
    return "Repeater";
}

} // bt
