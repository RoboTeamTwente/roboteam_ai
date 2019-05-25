//
// Created by thijs on 25-5-19.
//

#include "DribbleForwards.h"
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {
namespace control {


RobotCommand DribbleForwards::getRobotCommand(const world::Robot::RobotPtr &r, const Vector2 &targetP,
        const Angle &targetA) {

    updateForwardsProgress();
    return sendForwardsCommand();
}

void DribbleForwards::reset() {
    forwardsProgress = F_start;
}

void DribbleForwards::updateForwardsProgress() {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        printForwardsProgress();
    }

    // check if we still have ball
    targetAngle = (finalTargetPos - ball->pos).toAngle();
    Angle angleDifference = robot->angle - targetAngle;

    // update forwards progress
    switch (forwardsProgress) {
    case F_turning: {
        targetAngle = (finalTargetPos - ball->pos).toAngle();
        if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
            lockedAngle = targetAngle;
            forwardsProgress = F_approaching;
        }
        return;
    }
    case F_approaching: {
        if (fabs(angleDifference) > angleErrorMargin) {
            forwardsProgress = F_turning;
            return;
        }
        if (robot->hasBall()) {
            F_forwardsDribbleLine = {robot->pos, finalTargetPos};
            forwardsProgress = F_dribbleForward;
            return;
        }
        return;
    }
    case F_dribbleForward: {
        if (! robot->hasBall()) {
            forwardsProgress = F_approaching;
            return;
        }
        if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "we do not have a ball yet" << std::endl;
        }
        Angle offsetAngle = (finalTargetPos - robot->pos).toAngle() - robot->angle;
        double maxOffsetAngle = M_PI*0.05;
        if (fabs(offsetAngle) > maxOffsetAngle) {
            forwardsProgress = F_start;
            return;
        }
        if ((ball->pos - finalTargetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy) {
            forwardsProgress = F_success;
            return;
        }

        return;
    }
    case F_fail:
    case F_start:
    default:return;
    case F_success: {
        if ((ball->pos - finalTargetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy) {
            return;
        }
        forwardsProgress = F_start;
    }

    }
}

RobotCommand DribbleForwards::F_startTravelForwards() {
    lockedAngle = Angle();
    F_forwardsDribbleLine = {};
    forwardsProgress = F_turning;
    return F_sendTurnCommand();
}



RobotCommand DribbleForwards::sendForwardsCommand() {
    switch (forwardsProgress) {
    case F_start:return F_startTravelForwards();
    case F_turning:return F_sendTurnCommand();
    case F_approaching:return F_sendApproachCommand();
    case F_dribbleForward:return F_sendDribbleForwardsCommand();
    case F_success:return F_sendSuccessCommand();
    case F_fail: {
        forwardsProgress = F_start;
        return {};
    }
    }
}

RobotCommand DribbleForwards::F_sendTurnCommand() {
    if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
        lockedAngle = targetAngle;
    }
    targetPos = finalTargetPos;
    return rotateWithBall(rotateAroundBall);
}

RobotCommand DribbleForwards::F_sendApproachCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.vel = ballToRobot.stretchToLength(maxForwardsVelocity);
    command.angle = lockedAngle;
    return command;
}

RobotCommand DribbleForwards::F_sendDribbleForwardsCommand() {
    RobotCommand command;
    command.dribbler = 8;
    command.angle = lockedAngle;
    command.vel = lockedAngle.toVector2(maxForwardsVelocity);

    // check if the robot is still on the virtual line from ball->pos to the target
    if (control::ControlUtils::distanceToLine(robot->pos,
            F_forwardsDribbleLine.first, F_forwardsDribbleLine.second) > errorMargin*2.5) {
        forwardsProgress = F_turning;
    }

    // check if the ball is not too far right or too far left of the robot, and try to compensate for that
    if (ball->visible) {
        Angle ballAngleRelativeToRobot = robotToBall.toAngle() - robot->angle;
        command.vel += (robot->angle + M_PI_2).toVector2(ballAngleRelativeToRobot);
    }

    // limit velocity close to the target
    double distanceRemainingSquared = (finalTargetPos - robot->pos).length2();
    if (distanceRemainingSquared < 1.0) {
        command.vel.stretchToLength(distanceRemainingSquared*0.3);
    }

    return command;
}

RobotCommand DribbleForwards::F_sendSuccessCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.angle = lockedAngle;
    return command;
}

void DribbleForwards::printForwardsProgress() {
    std::stringstream ss;
    ss << "forwards progress:                  ";
    switch (forwardsProgress) {
    case F_start:ss << "start";
        break;
    case F_turning:ss << "turning";
        break;
    case F_approaching:ss << "approaching";
        break;
    case F_dribbleForward:ss << "dribble forwards";
        break;
    case F_success:ss << "success";
        break;
    case F_fail:ss << "fail";
        break;
    }
    std::cout << ss.str() << std::endl;
}

DribbleForwards::ForwardsProgress DribbleForwards::getForwardsProgression() {
    return forwardsProgress;
}

}
}
}