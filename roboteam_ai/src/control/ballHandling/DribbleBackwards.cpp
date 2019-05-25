//
// Created by thijs on 25-5-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>

#include "DribbleBackwards.h"
#include "DribbleForwards.h"
#include "RotateAroundBall.h"
#include "RotateAroundRobot.h"

namespace rtt {
namespace ai {
namespace control {

RobotCommand DribbleBackwards::getRobotCommand(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA) {
    robot = r;
    ball = world::world->getBall();
    finalTargetAngle = targetA;
    targetAngle = targetA;
    finalTargetPos = targetP;
    targetPos = targetP;
    updateBackwardsProgress();
    return sendBackwardsCommand();
}

void DribbleBackwards::reset() {
    backwardsProgress = B_start;
}

void DribbleBackwards::updateBackwardsProgress() {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        printBackwardsProgress();
    }

    if (backwardsProgress != B_dribbling) waitingTicks = 25;

    // check if we still have ball
    if (backwardsProgress != B_overshooting &&
            backwardsProgress != B_dribbling &&
            backwardsProgress != B_dribbleBackwards) {

        B_approachPosition = ball->pos + (robot->pos - ball->pos).stretchToLength(0.05);
    }
    if ((ball->pos - robot->pos).length2() > 0.5) {
        backwardsProgress = B_start;
        return;
    }
    targetAngle = (ball->pos - finalTargetPos).toAngle();
    Angle angleDifference = robot->angle - targetAngle;

    // update backwards progress
    switch (backwardsProgress) {
    case B_turning: {
        targetAngle = (ball->pos - finalTargetPos).toAngle();
        if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
            lockedAngle = targetAngle;
            backwardsProgress = B_approaching;
        }
        return;
    }
    case B_approaching: {
        if (fabs(angleDifference) > angleErrorMargin) {
            backwardsProgress = B_turning;
            return;
        }
        if (robot->hasBall()) {
            backwardsProgress = B_overshooting;
            return;
        }
        return;
    }
    case B_overshooting: {
        if (! robot->hasBall()) {
            backwardsProgress = B_start;
            return;
        }
        double overshoot = 0.06;
        if (((B_approachPosition - robot->pos)).length() < overshoot) {
            backwardsProgress = B_dribbling;
            return;
        }
        return;
    }
    case B_dribbling: {
        if (! robot->hasBall()) {
            backwardsProgress = B_approaching;
            return;
        }
        if (-- waitingTicks < 0) {
            B_backwardsDribbleLine = {robot->pos, finalTargetPos};
            backwardsProgress = B_dribbleBackwards;
            return;
        }
        return;
    }
    case B_dribbleBackwards: {
        if (! robot->hasBall()) {

            backwardsProgress = B_approaching;
            return;
        }
        Vector2 finalTargetToRobot = robot->pos - finalTargetPos;
        Angle offsetAngle = finalTargetToRobot.toAngle() - robot->angle;
        double maxOffsetAngle = M_PI*0.05;
        if (fabs(offsetAngle) > maxOffsetAngle) {
            if (finalTargetToRobot.length2() > Constants::ROBOT_RADIUS()*1.2) {
                backwardsProgress = B_start;
                return;
            }
        }
        if ((ball->pos - finalTargetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy) {
            backwardsProgress = B_success;
            return;
        }
        return;
    }
    case B_fail:
    case B_start:
    default:return;
    case B_success: {
        if ((ball->pos - finalTargetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy) {
            return;
        }
        backwardsProgress = B_start;
    }
    }
}

RobotCommand DribbleBackwards::sendBackwardsCommand() {
    switch (backwardsProgress) {
    case B_start: return B_startTravelBackwards();
    case B_turning:return B_sendTurnCommand();
    case B_approaching:return B_sendApproachCommand();
    case B_overshooting:return B_sendOvershootCommand();
    case B_dribbling:return B_sendDribblingCommand();
    case B_dribbleBackwards:return B_sendDribbleBackwardsCommand();
    case B_success:return B_sendSuccessCommand();
    case B_fail: {
        backwardsProgress = B_start;
        return {};
    }
    default: return {};
    }
}

RobotCommand DribbleBackwards::B_startTravelBackwards() {
    B_approachPosition = Vector2();
    lockedAngle = Angle();
    B_backwardsDribbleLine = {};
    backwardsProgress = B_turning;
    return B_sendTurnCommand();
}

RobotCommand DribbleBackwards::B_sendTurnCommand() {
    if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
        lockedAngle = targetAngle;
    }
    targetPos = finalTargetPos;
    return rotateAroundBall->getRobotCommand(robot, targetPos, targetAngle);
}

RobotCommand DribbleBackwards::B_sendApproachCommand() {
    RobotCommand command;
    command.dribbler = 1;
    command.vel = (ball->pos - robot->pos).stretchToLength(maxVel);
    command.angle = lockedAngle;
    return command;
}

RobotCommand DribbleBackwards::B_sendOvershootCommand() {
    RobotCommand command;
    command.dribbler = 8;
    command.vel = (B_approachPosition - robot->pos).stretchToLength(maxVel);
    command.angle = lockedAngle;
    return command;
}

RobotCommand DribbleBackwards::B_sendDribblingCommand() {
    RobotCommand command;
    command.dribbler = 16;
    command.angle = lockedAngle;
    return command;
}

RobotCommand DribbleBackwards::B_sendDribbleBackwardsCommand() {
    RobotCommand command;
    command.dribbler = 24;
    command.angle = lockedAngle;
    command.vel = lockedAngle.toVector2(- maxVel);

    // check if the robot is still on the virtual line from ball->pos to the target
    if (control::ControlUtils::distanceToLine(robot->pos,
            B_backwardsDribbleLine.first, B_backwardsDribbleLine.second) > errorMargin*2.5) {
        backwardsProgress = B_turning;
    }

    // check if the ball is not too far right or too far left of the robot, and try to compensate for that
    if (ball->visible) {
        Angle ballAngleRelativeToRobot = (ball->pos - robot->pos).toAngle() - robot->angle;
        command.vel += (robot->angle + M_PI_2).toVector2(ballAngleRelativeToRobot);
    }

    return command;
}

RobotCommand DribbleBackwards::B_sendSuccessCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.angle = lockedAngle;
    return command;
}

void DribbleBackwards::printBackwardsProgress() {
    std::stringstream ss;
    ss << "backwards progress:                  ";
    switch (backwardsProgress) {
    case B_overshooting:ss << "overshooting";
        break;
    case B_dribbling:ss << "dribbling";
        break;
    case B_dribbleBackwards:ss << "dribbleBackwards";
        break;
    case B_success:ss << "success";
        break;
    case B_fail:ss << "fail";
        break;
    case B_start:ss << "start";
        break;
    case B_turning:ss << "turning";
        break;
    case B_approaching:ss << "approaching";
        break;
    }
    std::cout << ss.str() << std::endl;
}

DribbleBackwards::BackwardsProgress DribbleBackwards::getBackwardsProgression() {
    return backwardsProgress;
}

DribbleBackwards::DribbleBackwards(double errorMargin, double angularErrorMargin, double ballPlacementAccuracy, double maxVel)
        :waitingTicks(0), errorMargin(errorMargin), angleErrorMargin(angularErrorMargin),
         ballPlacementAccuracy(ballPlacementAccuracy), maxVel(maxVel) {
    rotateAroundBall = new RotateAroundBall();
    rotateAroundRobot = new RotateAroundRobot();
}

}
}
}
