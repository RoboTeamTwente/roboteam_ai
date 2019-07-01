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
    forwardDirection = properties->getBool("forwardDirection");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else if (properties->getBool("ballPlacement")){
        targetPos = coach::g_ballPlacement.getBallPlacementPos();
    }

    else {
        ROS_ERROR("Dribble Initialize -> No maxTicks set!");
    }

    if (properties->hasDouble("distance")) {
        distance = properties->getDouble("distance");
        double targetAngle;
        if (forwardDirection) {
            targetAngle = robot->angle;
        } else {
            targetAngle = control::ControlUtils::constrainAngle(robot->angle - M_PI);
        }
        targetPos = (Vector2)robot->pos + Vector2({distance, 0}).rotate(targetAngle);
    }

    count = 0;
}

Dribble::Status Dribble::onUpdate() {
    if (properties->getBool("ballPlacement")){
        targetPos = coach::g_ballPlacement.getBallPlacementPos();
    }

    auto c = robot->getBallHandlePosControl()->getRobotCommand(robot, targetPos, robot->angle, control::BallHandlePosControl::TravelStrategy::FORWARDS);

    if(robot->getBallHandlePosControl()->getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    count++;
    if(count >= maxTicks && properties->hasInt("maxTicks")) {
        return Status::Failure;
    }

    command = c.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt
