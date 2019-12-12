/*
 * Return SUCCESS if the ball is close to the border
 * properties: 
 * - margin: the distance from the sides of the field which are 'close' to the border
 * - layingStill: if true, the ball has to lay still as well to return SUCCESS
 */

#include "conditions/IsBallCloseToBorder.h"
#include <world/Ball.h>

namespace rtt::ai {

IsBallCloseToBorder::IsBallCloseToBorder(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsBallCloseToBorder::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }
    ballShouldLayStill = properties->getBool("layingStill");
}

bt::Node::Status IsBallCloseToBorder::onUpdate() {
    if (properties->getBool("corner")) {
        auto fieldMsg = field->get_field();
        double xDiff = fieldMsg.get(FIELD_LENGTH) / 2 - abs(ball->getPos().x);
        double yDiff = fieldMsg.get(FIELD_WIDTH) / 2 - abs(ball->getPos().y);

        if (xDiff >= margin || yDiff >= margin) {
            return Status::Failure;
        }
    } 
    else if (field->pointIsInField(ball->getPos(), static_cast<float>(margin))) {
        return Status::Failure;
    }

    if (ballShouldLayStill) {
        bool ballIsLayingStill = Vector2(ball->getVel()).length() <= Constants::BALL_STILL_VEL();
        return ballIsLayingStill ? Status::Success : Status::Failure;
    } 
    return Status::Success;
}

} // rtt
