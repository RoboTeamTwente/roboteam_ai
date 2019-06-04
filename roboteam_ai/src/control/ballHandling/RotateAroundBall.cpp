#include <utility>

//
// Created by thijs on 25-5-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>

#include "RotateAroundBall.h"

namespace rtt {
namespace ai {
namespace control {

RobotCommand RotateAroundBall::getRobotCommand(const world::Robot::RobotPtr &robot, const Vector2 &targetP,
        const Angle &targetA) {

    auto ball = world::world->getBall();
    targetAngle = targetA;
    targetPos = targetP;

    RobotCommand robotCommand;
    Angle deltaAngle = targetA - robot->angle;
    double targetVel;

    if (deltaAngle.getAngle() > M_PI_2) targetVel = maxVel;
    else if (deltaAngle.getAngle() < - M_PI_2) targetVel = - maxVel;
    else targetVel = deltaAngle.getAngle()*maxVel/M_PI_2;

    robotCommand.vel = (ball->pos - robot->pos).rotate(- M_PI_2).stretchToLength(targetVel) - previousVelocity*0.2;

    if ((ball->pos - robot->pos).length2() > maxBallDistance*maxBallDistance) {
        robotCommand.vel += (ball->pos - robot->pos) - (ball->pos - robot->pos).stretchToLength(targetBallDistance);
    }

    robotCommand.angle = (ball->pos - robot->pos).toAngle();
    robotCommand.dribbler = 0;

    previousVelocity = robotCommand.vel;
    return robotCommand;
}
}
}
}