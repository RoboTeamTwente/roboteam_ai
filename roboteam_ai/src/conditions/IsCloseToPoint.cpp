//
// Created by robzelluf on 1/23/19.
//

#include "IsCloseToPoint.h"

namespace rtt {
namespace ai {

IsCloseToPoint::IsCloseToPoint(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsCloseToPoint::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    } else margin = 0.0;

    if (properties->hasBool("ball")) {
        ballPos = true;
        position = ball->pos;
    } else {
        position = properties->getVector2("position");
        ballPos = false;
    }
}

IsCloseToPoint::Status IsCloseToPoint::onUpdate() {
    if (ballPos) {
        position = ball->pos;
    }

    double deltaPos = (position - robot->pos).length();

    if (deltaPos >= margin) {
        return Status::Failure;
    }
    else {
        return Status::Success;
    }
}

std::string IsCloseToPoint::node_name() {return "IsCloseToPoint";}
}
}
