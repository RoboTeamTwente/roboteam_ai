//
// Created by mrlukasbos on 25-1-19.
//

#include <roboteam_ai/src/utilities/Field.h>
#include <roboteam_ai/src/utilities/World.h>
#include "BallOutOfField.h"

namespace rtt {
namespace ai {

BallOutOfField::BallOutOfField(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status BallOutOfField::update() {
    Vector2 ballPos;
    auto ball = World::getBall();
    if (ball) {
        ballPos = ball->pos;
    } else {
        return Status::Failure;
    }

    // return success if the ball is out of the field
    if (abs(ballPos.x) < Field::get_field().field_length / 2 &&
            abs(ballPos.y) < Field::get_field().field_width / 2) {
        return Status::Failure;
    } else {
        return Status::Success;
    }

}

std::string BallOutOfField::node_name() {return "BallOutOfField";}

} // ai
} // rtt