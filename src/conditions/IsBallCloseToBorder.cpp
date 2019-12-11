/*
 * Return SUCCESS if the ball is close to the border
 * properties: 
 * - margin: the distance from the sides of the field which are 'close' to the border
 * - layingStill: if true, the ball has to lay still as well to return SUCCESS
 */

#include "conditions/IsBallCloseToBorder.h"
#include <world/Ball.h>

namespace rtt {
namespace ai {

IsBallCloseToBorder::IsBallCloseToBorder(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsBallCloseToBorder::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }
    ballShouldLayStill = properties->getBool("layingStill");
}

bt::Node::Status IsBallCloseToBorder::onUpdate() {
    auto fieldMsg = FieldMessage::get_field();
    if (properties->getBool("corner")) {
        double xDiff = fieldMsg[FIELD_LENGTH] / 2 - abs(ball->getPos().x);
        double yDiff = fieldMsg[FIELD_WIDTH] / 2 - abs(ball->getPos().y);

        if (xDiff >= margin || yDiff >= margin) {
            return Status::Failure;
        }
    } 
    else if (world::FieldComputations::pointIsInField(fieldMsg, ball->getPos(), static_cast<float>(margin))) {
        return Status::Failure;
    }

    if (ballShouldLayStill) {
        bool ballIsLayingStill = Vector2(ball->getVel()).length() <= Constants::BALL_STILL_VEL();
        return ballIsLayingStill ? Status::Success : Status::Failure;
    } 
    return Status::Success;
}

} // ai
} // rtt
