//
// Created by robzelluf on 10/24/18.
//

#include "TheyHaveBall.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai {

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status TheyHaveBall::onUpdate() {
    if (World::theyHaveBall()) {
        return bt::Node::Status::Success;
    }
    return bt::Node::Status::Failure;
}

} // ai
} // rtt