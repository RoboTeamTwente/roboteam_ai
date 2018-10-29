
#include "Repeater.hpp"

namespace bt {

    Repeater::Repeater(int limit) : limit(limit) {}

    void Repeater::Initialize() {
        counter = 0;
    }

    Node::Status Repeater::Update() {
        while (1) {
            Node::append_status("[Repeater: executing child of type %s]", child->node_name().c_str());
            child->Tick();

            if (limit > 0 && ++counter == limit) {
                return Status::Success;
            }

            return Status::Running;
        }
    }

    std::string Repeater::node_name() {
        return "Repeater";
    }

} // bt
