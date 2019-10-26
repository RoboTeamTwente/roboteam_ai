//
// Created by rolf on 28/11/18.
//

#include <coach/BallplacementCoach.h>
#include "skills/Dribble.h"
namespace rtt {
namespace ai {

Dribble::Dribble(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

void Dribble::onInitialize() {

    //if false, robot will dribble to the position backwards with the ball.
    if (properties->hasBool("forwardDirection")) {
        forwardDirection = properties->getBool("forwardDirection") ?
                           control::BallHandlePosControl::TravelStrategy::FORWARDS :
                           control::BallHandlePosControl::TravelStrategy::BACKWARDS;
    }
    else {
        forwardDirection = control::BallHandlePosControl::TravelStrategy::FORWARDS;
    }

    if (properties->hasDouble("distance")) {
        distance = properties->getDouble("distance");
        Angle targetAngle;
        if (forwardDirection == control::BallHandlePosControl::TravelStrategy::FORWARDS) {
            targetAngle = robot->angle;
        }
        else {
            targetAngle = robot->angle - M_PI;
        }
        targetPos = (Vector2) robot->pos + Vector2({distance, 0}).rotate(targetAngle);
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

    auto c = robot->getBallHandlePosControl()->getRobotCommand(robot, targetPos, robot->angle, forwardDirection);

    if (robot->getBallHandlePosControl()->getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    count ++;
    if (count >= maxTicks && properties->hasInt("maxTicks")) {
        return Status::Failure;
    }

    command = c.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt
