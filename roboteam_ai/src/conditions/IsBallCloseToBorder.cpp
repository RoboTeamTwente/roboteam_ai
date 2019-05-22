/*
 * Return SUCCESS if the ball is close to the border
 * properties: 
 * - margin: the distance from the sides of the field which are 'close' to the border
 */

#include "IsBallCloseToBorder.h"

namespace rtt {
namespace ai {

IsBallCloseToBorder::IsBallCloseToBorder(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsBallCloseToBorder::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }
}

bt::Node::Status IsBallCloseToBorder::onUpdate() {
    if (world::field->pointIsInField(ball->pos, static_cast<float>(margin))) {
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
