//
// Created by rolf on 28/11/18.
//

#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "Dribble.h"
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
        Vector2 target = coach::g_ballPlacement.getBallPlacementAfterPos(robot);

        if ((targetPos - robot->pos).length() < 0.1) {
            return Status::Success;
        }

        command = robot->getNumtreePosControl()->getRobotCommand(robot, targetPos).makeROSCommand();

        // set robotcommands if they have not been set yet in gtpUpdate()


        publishRobotCommand();
        return Status::Running;
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
