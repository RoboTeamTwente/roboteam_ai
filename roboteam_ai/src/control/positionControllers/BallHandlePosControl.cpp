//
// Created by thijs on 18-12-18.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "BallHandlePosControl.h"

namespace rtt {
namespace ai {
namespace control {

//TODO: receive horizontal position of the ball relative to the robot using feedback <3
//TODO: delay compensation

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea)
        :canMoveInDefenseArea(canMoveInDefenseArea) {
    numTreePosController.setCanMoveInDefenseArea(canMoveInDefenseArea);
    numTreePosController.setAvoidBall(targetBallDistance*0.95);

}

/// targetP is the target position of the BALL, targetA is the (final) target angle of the ROBOT
RobotCommand BallHandlePosControl::getPosVelAngle(const RobotPtr &r,
        const Vector2 &targetP, const Angle &targetA) {

    // update variables
    updateVariables(r, targetP, targetA);

    // check for ball
    if (! ball) {
        if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Can't control the ball with no ball" << std::endl;
        }
        return {};
    }

    // if the ball is at the targetposition
    if ((ball->pos - finalTargetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy) {
        backwardsProgress = B_start;
        forwardsProgress = F_start;
        if (ball->vel.length2() < errorMargin*errorMargin) {
            return rotateWithBall(rotateAroundBall);
        }
    }

    // if we do not have the ball yet, go get it
    double deltaPosSquared = (finalTargetPos - ball->pos).length2();
    bool ballIsFarFromTarget = deltaPosSquared > 0.5;

    bool robotDoesNotHaveBall = ! robot->hasBall();
    bool robotIsTooFarFromBall = ballToRobot.length2() > maxBallDistance*maxBallDistance;
    if ((robotDoesNotHaveBall && robotIsTooFarFromBall)) {
        return goToBall(ballIsFarFromTarget);
    }

    // check if we are doing something already
    if (backwardsProgress != B_start) return travelWithBall(backwards);
    if (forwardsProgress != F_start) return travelWithBall(forwards);


    // check if we are far from the final target
    if (ballIsFarFromTarget) {
        return travelWithBall(forwards);
    }

    // if the distance to the target is small
    return travelWithBall(backwards);
}

RobotCommand BallHandlePosControl::rotateWithBall(RotateStrategy rotateStrategy) {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        printRotateStrategy(rotateStrategy);
    }

    RobotCommand robotCommand;
    Angle deltaAngle = targetAngle - robot->angle;

    switch (rotateStrategy) {
    case rotateAroundBall: {
        double maxV = maxForwardsVelocity;
        double targetVel = deltaAngle.getAngle();
        targetVel = targetVel > M_PI_2 ? maxV :
                    targetVel < - M_PI_2 ? - maxV :
                    targetVel*maxV/M_PI_2;

        robotCommand.vel = robotToBall.rotate(- M_PI_2).stretchToLength(targetVel) - previousVelocity*0.2;

        if (robotToBall.length2() > maxBallDistance*maxBallDistance) {
            robotCommand.vel += robotToBall - robotToBall.stretchToLength(targetBallDistance);
        }

        robotCommand.angle = robotToBall.toAngle();
        robotCommand.dribbler = 0;
        return limitCommand(robotCommand);
    }
    case rotateAroundRobot: {
        int direction = targetAngle - robot->angle > 0.0 ? 1 : - 1;
        robotCommand.angle = Angle(robot->angle + maxAngularVelocity*direction);
        robotCommand.dribbler = 1;
        return limitCommand(robotCommand);
    }
    }
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        std::cout << "rotate case not handled" << std::endl;
    }
    return {};
}

RobotCommand BallHandlePosControl::travelWithBall(TravelStrategy travelStrategy) {

    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        printTravelStrategy(travelStrategy);
    }

    switch (travelStrategy) {
    case backwards: {
        updateBackwardsProgress();

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
        }
    }
    case forwards: {
        updateForwardsProgress();

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
    }

    std::cout << "travel case not handled" << std::endl;
    return {
    };
}

RobotCommand BallHandlePosControl::B_startTravelBackwards() {
    B_count = 0;
    B_approachPosition = Vector2();
    B_lockedAngle = Angle();
    B_backwardsDribbleLine = {};
    backwardsProgress = B_turning;
    return B_sendTurnCommand();
}

RobotCommand BallHandlePosControl::B_sendTurnCommand() {
    if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
        B_lockedAngle = targetAngle;
    }
    targetPos = finalTargetPos;
    return rotateWithBall(rotateAroundBall);
}

RobotCommand BallHandlePosControl::B_sendApproachCommand() {
    RobotCommand command;
    command.dribbler = 1;
    command.vel = robotToBall.stretchToLength(maxBackwardsVelocity);
    command.angle = B_lockedAngle;
    return command;
}

RobotCommand BallHandlePosControl::B_sendOvershootCommand() {
    RobotCommand command;
    command.dribbler = 1;
    command.vel = (B_approachPosition - robot->pos).stretchToLength(maxBackwardsVelocity);
    command.angle = B_lockedAngle;
    return command;
}

RobotCommand BallHandlePosControl::B_sendDribblingCommand() {
    RobotCommand command;
    command.dribbler = 1;
    command.angle = B_lockedAngle;
    return command;
}

RobotCommand BallHandlePosControl::B_sendDribbleBackwardsCommand() {
    RobotCommand command;
    command.dribbler = 1;
    command.angle = B_lockedAngle;
    command.vel = B_lockedAngle.toVector2(- maxBackwardsVelocity);

    // check if the robot is still on the virtual line from ball->pos to the target
    if (control::ControlUtils::distanceToLine(robot->pos,
            B_backwardsDribbleLine.first, B_backwardsDribbleLine.second) > errorMargin*2.5) {
        backwardsProgress = B_turning;
    }

    // check if the ball is not too far right or too far left of the robot, and try to compensate for that
    if (ball->visible) {
        Angle ballAngleRelativeToRobot = robotToBall.toAngle() - robot->angle;
        command.vel += (robot->angle + M_PI_2).toVector2(ballAngleRelativeToRobot);
    }

    return command;
}

RobotCommand BallHandlePosControl::B_sendSuccessCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.angle = B_lockedAngle;
    return command;
}


RobotCommand BallHandlePosControl::F_sendTurnCommand() {
    if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
        F_lockedAngle = targetAngle;
    }
    targetPos = finalTargetPos;
    return rotateWithBall(rotateAroundBall);
}

RobotCommand BallHandlePosControl::F_sendApproachCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.vel = ballToRobot.stretchToLength(maxForwardsVelocity);
    command.angle = F_lockedAngle;
    return command;
}

RobotCommand BallHandlePosControl::F_sendDribbleForwardsCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.angle = F_lockedAngle;
    command.vel = F_lockedAngle.toVector2(maxForwardsVelocity);

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

RobotCommand BallHandlePosControl::F_sendSuccessCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.angle = F_lockedAngle;
    return command;
}

void BallHandlePosControl::updateBackwardsProgress() {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        printBackwardsProgress();
    }

    // check if we still have ball
    if (backwardsProgress != B_overshooting && backwardsProgress != B_dribbling) {
        B_approachPosition = ball->pos + ballToRobot.stretchToLength(0.05);
    }
    if (robotToBall.length2() > 0.5) {
        backwardsProgress = B_fail;
        return;
    }
    targetAngle = (ball->pos - finalTargetPos).toAngle();
    Angle angleDifference = robot->angle - targetAngle;
    if (backwardsProgress != B_dribbling) B_count = 0;

    // update backwards progress
    switch (backwardsProgress) {
    case B_turning: {
        targetAngle = (ball->pos - finalTargetPos).toAngle();
        if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
            B_lockedAngle = targetAngle;
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
        B_count ++;
        int dribbleCount = 25;
        if (B_count > dribbleCount) {
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
        Angle offsetAngle = (robot->pos - finalTargetPos).toAngle() - robot->angle;
        double maxOffsetAngle = M_PI*0.05;
        if (fabs(offsetAngle) > maxOffsetAngle) {
            backwardsProgress = B_start;
            return;
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

void BallHandlePosControl::updateForwardsProgress() {
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
            F_lockedAngle = targetAngle;
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

RobotCommand BallHandlePosControl::limitCommand(RobotCommand command) {

    // velocity limiter
    command.vel = control::ControlUtils::velocityLimiter(command.vel);

    // acceleration limiter
    double maxAcc = control::ControlUtils::calculateMaxAcceleration(command.vel, command.angle);
    command.vel = control::ControlUtils::accelerationLimiter(command.vel, maxAcc, previousVelocity.length());
    previousVelocity = command.vel;
    return command;
}

RobotCommand BallHandlePosControl::goToBall(bool ballIsFarFromTarget) {

    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        std::cout << "we do not have a ball yet" << std::endl;
    }
    backwardsProgress = B_start;
    forwardsProgress = F_start;
    RobotCommand robotCommand;
    Vector2 target;
    Vector2 ballToTarget = finalTargetPos - ball->pos;
    if (ballIsFarFromTarget) {
        target = ball->pos + ballToTarget.stretchToLength(- maxBallDistance);
    }
    else {
        target = ball->pos + ballToTarget.stretchToLength(maxBallDistance);
    }

    auto pva = numTreePosController.getPosVelAngle(robot, target);
    robotCommand.angle = robotToBall.toAngle();
    robotCommand.vel = pva.vel;
    return limitCommand(robotCommand);
}

void BallHandlePosControl::updateVariables(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA) {
    double expectedDelay = 0.04;
    ball = world::world->getBall();
    robot = world::world->getFutureRobot(r, expectedDelay);
    targetPos = targetP;
    finalTargetPos = targetP;
    targetAngle = targetA;
    finalTargetAngle = targetA;
    robotToBall = ball->pos - robot->pos;
    ballToRobot = robot->pos - ball->pos;
}

RobotCommand BallHandlePosControl::F_startTravelForwards() {
    F_lockedAngle = Angle();
    F_forwardsDribbleLine = {};
    forwardsProgress = F_turning;
    return F_sendTurnCommand();
}

void BallHandlePosControl::printForwardsProgress() {
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

void BallHandlePosControl::printBackwardsProgress() {
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

void BallHandlePosControl::printTravelStrategy(TravelStrategy strategy) {
    std::stringstream ss;
    ss << "travel with strategy: ";
    switch (strategy) {
    case forwards:ss << "forwards";
        break;
    case backwards:ss << "backwards";
        break;
    }
    std::cout << ss.str() << std::endl;
}

void BallHandlePosControl::printRotateStrategy(RotateStrategy strategy) {
    std::stringstream ss;
    ss << "rotating with strategy: ";
    switch (strategy) {
    case rotateAroundBall:ss << "aroundBall";
        break;
    case rotateAroundRobot:ss << "aroundRobot";
        break;
    }
    std::cout << ss.str() << std::endl;
}

} //control
} //ai
} //rtt