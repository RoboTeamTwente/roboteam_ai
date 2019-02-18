//
// Created by robzelluf on 2/18/19.
//

#include "IsBallCloseToBorder.h"

namespace rtt{
namespace ai {

IsBallCloseToBorder::IsBallCloseToBorder(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsBallCloseToBorder::initialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }
}

bt::Node::Status IsBallCloseToBorder::update() {
    Vector2 ballPos = World::getBall()->pos;
    if (!Field::pointIsInField(ballPos, static_cast<float>(margin))) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

std::string IsBallCloseToBorder::node_name() {return "IsBallCloseToBorder";}

}
}
