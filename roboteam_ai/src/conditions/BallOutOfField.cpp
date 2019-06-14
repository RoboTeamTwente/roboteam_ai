/* 
 * Return SUCCESS if the ball is out of the field
 * otherwise FAILURE
 */

#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/world/Ball.h>
#include "BallOutOfField.h"

namespace rtt {
namespace ai {

BallOutOfField::BallOutOfField(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status BallOutOfField::onUpdate() {
    Vector2 ballPos;
    if (properties->hasDouble("secondsAhead")) {
        ballPos = ball->pos + ball->vel * properties->getDouble("secondsAhead");
    } else {
        ballPos = ball->pos;
    }

    // return success if the ball is out of the field
    if (!world::field->pointIsInField(ballPos)) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

} // ai
} // rtt