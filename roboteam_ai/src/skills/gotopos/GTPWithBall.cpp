#include <utility>

#include <utility>
#include <roboteam_ai/src/coach/BallplacementCoach.h>

//
// Created by thijs on 26-4-19.
//

#include "GTPWithBall.h"

namespace rtt {
namespace ai {

GTPWithBall::GTPWithBall(string name, bt::Blackboard::Ptr blackboard)
        : Skill(std::move(name), std::move(blackboard)) { }

void GTPWithBall::onInitialize() {
    targetType = stringToTargetType(properties->getString("type"));
    updateTarget();
}

Skill::Status GTPWithBall::onUpdate() {
    updateTarget();
    command = ballHandlePosControl.getRobotCommand(robot, targetPos, targetAngle).makeRobotCommand();
    publishRobotCommand();
    return Status::Running;
}

void GTPWithBall::onTerminate(Skill::Status s) {
    command.dribbler = 0;
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = robot->angle;
    publishRobotCommand();
}

GTPWithBall::TargetType GTPWithBall::stringToTargetType(const std::string &string) {
    return ballPlacement;
}

void GTPWithBall::updateTarget() {
    switch (targetType) {
    default:return;
    case ballPlacement: {
        targetPos = coach::g_ballPlacement.getBallPlacementPos();
        Vector2 delta = (targetPos - ball->pos);
        if (fabs(robot->angle - delta.toAngle()) < M_PI_2) {
            targetAngle = delta;
        }
        else {
            targetAngle = delta + M_PI;
        }
        return;
    }
    }
}

}
}