//
// Created by robzelluf on 5/14/19.
//

#include "IsBallMovingFast.h"

namespace rtt {
namespace ai {

IsBallMovingFast::IsBallMovingFast(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsBallMovingFast::onInitialize() {
    if (properties->hasDouble("minVel")) {
        minVelocity = properties->getDouble("minVel");
    } else {
        minVelocity = Constants::BALL_MOVING_FAST_VEL();
    }
}

IsBallMovingFast::Status IsBallMovingFast::onUpdate() {
    if (ball->vel.length() > minVelocity) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

}
}
