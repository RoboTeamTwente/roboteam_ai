/*
 * Return SUCCESS if the ball is close to the border
 * properties: 
 * - margin: the distance from the sides of the field which are 'close' to the border
 * - layingStill: if true, the ball has to lay still as well to return SUCCESS
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
    ballShouldLayStill = properties->getBool("layingStill");
}

bt::Node::Status IsBallCloseToBorder::onUpdate() {
    if (properties->getBool("corner")) {
        auto field = world::field->get_field();
        double xDiff = field.field_length / 2 - abs(ball->pos.x);
        double yDiff = field.field_width / 2 - abs(ball->pos.y);

        if (xDiff >= margin || yDiff >= margin) {
            return Status::Failure;
        }
    } 
    else if (world::field->pointIsInField(ball->pos, static_cast<float>(margin))) {
        return Status::Failure;
    }

    if (ballShouldLayStill) {
        bool ballIsLayingStill = Vector2(ball->vel).length() <= Constants::BALL_STILL_VEL();
        return ballIsLayingStill ? Status::Success : Status::Failure;
    } 
    return Status::Success;
}

} // ai
} // rtt
