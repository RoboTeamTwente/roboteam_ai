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

    if (properties->hasInt("maxTicks")) {
        maxTicks = properties->getInt("maxTicks");
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

    stoppingAngle = robot->angle; // default to the current angle
    initialAngle = robot->angle;
}

Dribble::Status Dribble::onUpdate() {
    if (properties->getBool("ballPlacement")){
        targetPos = coach::g_ballPlacement.getBallPlacementPos();
    }

    if(ballHandlePosControl.getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    count++;
    if(count >= maxTicks && properties->hasInt("maxTicks")) {
        return Status::Failure;
    }

    auto c = ballHandlePosControl.getRobotCommand(robot, targetPos, robot->angle, control::BallHandlePosControl::TravelStrategy::FORWARDS);
    command = c.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

void Dribble::onTerminate(Status s) {
    command.w = robot->angle;
    if (properties->getBool("dribbleOnTerminate")){
        command.dribbler=  20;
    } else{
        command.dribbler = 0;
    }
    command.x_vel = 0;
    command.y_vel = 0;

    count=0;
    stoppingAngle = robot->angle; // default to the current angle
    initialAngle = robot->angle;
    publishRobotCommand();
}

} // ai
} // rtt
