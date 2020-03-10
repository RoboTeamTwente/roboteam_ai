//
// Created by thijs on 25-5-19.
//

#include "control/ball-handling/RotateAroundBall.h"
#include <interface/api/Input.h>
#include "world_new/World.hpp"

namespace rtt::ai::control {

RobotCommand RotateAroundBall::getRobotCommand(world_new::view::RobotView robot, const Vector2 &targetP, const Angle &targetA) {
    auto ball = world_new::World::instance()->getWorld()->getBall().value();
    targetAngle = targetA;
    targetPos = targetP;

    RobotCommand robotCommand;
    Angle deltaAngle = targetA - (ball->getPos() - robot->getPos()).toAngle();
    double targetVel;

    if (deltaAngle.getAngle() > 1.0)
        targetVel = maxVel;
    else if (deltaAngle.getAngle() < -1.0)
        targetVel = -maxVel;
    else
        targetVel = deltaAngle.getAngle() * maxVel;
    auto previousVel = robot->getPidPreviousVel().length();

    targetVel = targetVel * 1.3 - previousVel * 0.3;
    robotCommand.vel = (ball->getPos() - robot->getPos()).rotate(-M_PI_2).stretchToLength(targetVel);

    if ((ball->getPos() - robot->getPos()).length2() > maxBallDistance * maxBallDistance) {
        robotCommand.vel += (ball->getPos() - robot->getPos()) - (ball->getPos() - robot->getPos()).stretchToLength(targetBallDistance);
    }

    interface::Input::drawData(interface::Visual::BALL_HANDLING, {robotCommand.vel + robot->getPos(), robot->getPos()}, Qt::black, robot->getId(), interface::Drawing::ARROWS);

    robotCommand.angle = (ball->getPos() - robot->getPos()).toAngle();
    robotCommand.dribbler = 0;

    previousVelocity = robotCommand.vel;
    return robotCommand;
}
}  // namespace rtt::ai::control