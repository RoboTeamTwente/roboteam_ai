//
// Created by robzelluf on 1/21/19.
//

#include "IsBallOnOurSide.h"

namespace rtt {
namespace ai {

IsBallOnOurSide::IsBallOnOurSide(std::string name, bt::Blackboard::Ptr blackboard)
    :Condition(std::move(name), std::move(blackboard)) { };

void IsBallOnOurSide::onInitialize() {
    if (properties->hasBool("inField")) {
        inField = properties->getBool("inField");
    }

}

bt::Node::Status IsBallOnOurSide::onUpdate() {
    Vector2 ballPos = ball->pos;

    if (ballPos.x < 0) {
        if (inField) {
            if (abs(ballPos.x) < world::field->get_field().field_length / 2 &&
                abs(ballPos.y) < world::field->get_field().field_width / 2) {
                return Status::Success;
            } else return Status::Failure;
        } else return Status::Success;
    }
    else return Status::Failure;
    }

std::string IsBallOnOurSide::node_name() {return "IsBallOnOurSide";}

}
}

