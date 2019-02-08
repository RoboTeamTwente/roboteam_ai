//
// Created by robzelluf on 1/21/19.
//

#include "IsBallOnOurSide.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {

IsBallOnOurSide::IsBallOnOurSide(std::string name, bt::Blackboard::Ptr blackboard)
    :Condition(std::move(name), std::move(blackboard)) { };

void IsBallOnOurSide::initialize() {
    if (properties->hasBool("inField")) {
        inField = properties->getBool("inField");
    }

}

bt::Node::Status IsBallOnOurSide::update() {
    Vector2 ballPos;
    auto ball = World::getBall();
    if (ball) {
        ballPos = ball->pos;
    }
    else return Status::Failure;

    if (ballPos.x < 0) {
        if (inField) {
            if (abs(ballPos.x) < Field::get_field().field_length / 2 &&
                abs(ballPos.y) < Field::get_field().field_width / 2) {
                return Status::Success;
            } else return Status::Failure;
        } else return Status::Success;
    }
    else return Status::Failure;
    }

std::string IsBallOnOurSide::node_name() {return "IsBallOnOurSide";}

}
}

