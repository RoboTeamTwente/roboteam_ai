/* 
 * Return SUCCESS if the ball is out of the field
 * otherwise FAILURE
 */

#include <roboteam_ai/src/world/Field.h>
#include "BallOutOfField.h"

namespace rtt {
namespace ai {

BallOutOfField::BallOutOfField(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status BallOutOfField::onUpdate() {
    Vector2 ballPos = ball->pos;

    // return success if the ball is out of the field
    if (abs(ballPos.x) < world::field->get_field().field_length / 2 &&
            abs(ballPos.y) < world::field->get_field().field_width / 2) {
        return Status::Failure;
    } 
    return Status::Success;
}

} // ai
} // rtt