//
// Created by thijs on 25-5-19.
//

#include "control/ball-handling/RotateAroundBall.h"

#include <interface/api/Input.h>

#include "control/ControlUtils.h"
#include "world/Ball.h"
#include "world/Robot.h"
#include "world/World.h"

namespace rtt::ai::control {

    RobotCommand RotateAroundBall::getRobotCommand(RobotPtr robot, const Vector2 &targetP, const Angle &targetA) {
        auto ball = world::world->getBall();
        targetAngle = targetA;
        targetPos = targetP;

        RobotCommand robotCommand;
        Angle deltaAngle = targetA - (ball->getPos() - robot->pos).toAngle();
        double targetVel;

        if (deltaAngle.getAngle() > 1.0)
            targetVel = maxVel;
        else if (deltaAngle.getAngle() < -1.0)
            targetVel = -maxVel;
        else
            targetVel = deltaAngle.getAngle() * maxVel;
        auto previousVel = robot->getPidPreviousVel().length();

        targetVel = targetVel * 1.3 - previousVel * 0.3;
        robotCommand.vel = (ball->getPos() - robot->pos).rotate(-M_PI_2).stretchToLength(targetVel);

        if ((ball->getPos() - robot->pos).length2() > maxBallDistance * maxBallDistance) {
            robotCommand.vel += (ball->getPos() - robot->pos) - (ball->getPos() - robot->pos).stretchToLength(targetBallDistance);
        }

        interface::Input::drawData(interface::Visual::BALL_HANDLING, {robotCommand.vel + robot->pos, robot->pos}, Qt::black, robot->id, interface::Drawing::ARROWS);

        robotCommand.angle = (ball->getPos() - robot->pos).toAngle();
        robotCommand.dribbler = 0;

        previousVelocity = robotCommand.vel;
        return robotCommand;
    }
}  // namespace rtt::ai::control