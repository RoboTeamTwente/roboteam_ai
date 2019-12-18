/* 
 * Return SUCCESS if the ball is out of the field
 * otherwise FAILURE
 */

#include <world/FieldComputations.h>
#include <world/Ball.h>
#include "conditions/BallOutOfField.h"

namespace rtt {
namespace ai {

BallOutOfField::BallOutOfField(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status BallOutOfField::onUpdate() {
    Vector2 ballPos;
    if (properties->hasDouble("secondsAhead")) {
        ballPos = ball->getPos() + ball->getVel() * properties->getDouble("secondsAhead");
    } else {
        ballPos = ball->getPos();
    }

    // return success if the ball is out of the field
    if (!FieldComputations::pointIsInField(*field, ballPos)) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

} // ai
} // rtt