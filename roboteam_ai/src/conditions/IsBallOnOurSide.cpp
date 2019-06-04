/*
* return SUCCESS if the ball is on our side
* properties:
* - inField: if true, the ball also has to be in the field to return SUCCESS
*/

#include "IsBallOnOurSide.h"

namespace rtt {
namespace ai {

IsBallOnOurSide::IsBallOnOurSide(std::string name, bt::Blackboard::Ptr blackboard)
    :Condition(std::move(name), std::move(blackboard)) { };

void IsBallOnOurSide::onInitialize() {
    inField = properties->getBool("inField");    
}

bt::Node::Status IsBallOnOurSide::onUpdate() {
    Vector2 ballPos = ball->pos;

    if (ballPos.x < 0) {
        if (inField) {
            if (abs(ballPos.x) < world::field->get_field().field_length / 2 &&
                abs(ballPos.y) < world::field->get_field().field_width / 2) {
                return Status::Success;
            }
            return Status::Failure;
        } 
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt

