//
// Created by rolf on 28/11/18.
//

#include <skills/Dribble.h>
#include <coach/BallplacementCoach.h>

#include <utility>

namespace rtt::ai {

Dribble::Dribble(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Dribble::onInitialize() {
    // if false, robot will dribble to the position backwards with the ball.
    if (properties->hasBool("forwardDirection")) {
        forwardDirection =
            properties->getBool("forwardDirection") ? control::BallHandlePosControl::TravelStrategy::FORWARDS : control::BallHandlePosControl::TravelStrategy::BACKWARDS;
    } else {
        forwardDirection = control::BallHandlePosControl::TravelStrategy::FORWARDS;
    }

    if (properties->hasDouble("distance")) {
        distance = properties->getDouble("distance");
        Angle targetAngle{};
        if (forwardDirection == control::BallHandlePosControl::TravelStrategy::FORWARDS) {
            targetAngle = robot->get()->getAngle();
        } else {
            targetAngle = robot->get()->getAngle() - M_PI;
        }
        targetPos = (Vector2)robot->get()->getPos() + Vector2({distance, 0}).rotate(targetAngle);
    }

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    if (properties->getBool("ballPlacement")) {
        targetPos = coach::g_ballPlacement.getBallPlacementPos();
    }

    count = 0;
}

Dribble::Status Dribble::onUpdate() {
    if (properties->getBool("ballPlacement")) {
        targetPos = coach::g_ballPlacement.getBallPlacementPos();
    }

    auto c = robot->getControllers().getBallHandlePosController()->getRobotCommand(robot->get()->getId(), targetPos, robot->get()->getAngle(), forwardDirection);

    if (robot->getControllers().getBallHandlePosController()->getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    count++;
    if (count >= maxTicks && properties->hasInt("maxTicks")) {
        return Status::Failure;
    }

    command = c.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

}  // namespace rtt::ai
