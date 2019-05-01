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

RobotCommand BallHandlePosControl::getPosVelAngle(const RobotPtr &r,
        const Vector2 &targetP, const Angle &targetA) {

    // update variables
    updateVariables(r, targetP, targetA);

    // check for ball
    if (! ball) {
        if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Can't control the ball with no ball, you stupid" << std::endl;
        }
        return {};
    }

    // check if we are doing something already
    if (backwardsProgress != B_start) return travelWithBall(backwards);
    if (forwardsProgress != F_start) return travelWithBall(forwards);

    // if we do not have the ball yet, go get it
    bool robotDoesNotHaveBall = ! robot->hasBall();
    bool robotIsTooFarFromBall = ballToRobot.length2() > maxBallDistance*maxBallDistance;
    if ((robotDoesNotHaveBall || robotIsTooFarFromBall)) {
        goToBall(robotDoesNotHaveBall, robotIsTooFarFromBall);
    }

    // check if we are far from the final target
    double deltaPosSquared = (finalTargetPos - ball->pos).length2();
    if (deltaPosSquared > 0.5) {
        return travelWithBall(forwards);
    }

    // if the distance to the target is small
    travelWithBall(backwards);
}

RobotCommand BallHandlePosControl::rotateWithBall(RotateStrategy rotateStrategy) {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        std::stringstream ss;
        ss << "rotating with strategy: ";
        switch (rotateStrategy) {
        case rotateAroundBall:ss << "aroundBall";
            break;
        case rotateAroundRobot:ss << "aroundRobot";
            break;
        case fastest:ss << "fastest";
            break;
        case safest:ss << "safest";
            break;
        case defaultRotate:ss << "default";
            break;
        }
        std::cout << ss.str() << std::endl;
    }

    RobotCommand robotCommand;
    Angle deltaAngle = targetAngle - robot->angle;
    Angle compensationAngle = 0;
    Vector2 compensationVelocity = Vector2();
    if (ball->visible) {
        compensationAngle = robotToBall.toAngle() - robot->angle;
        compensationVelocity = (robot->angle + M_PI_2).toVector2(compensationAngle);
    }

    switch (rotateStrategy) {
    case rotateAroundBall: {
        double maxV = 0.4;
        double targetVel = deltaAngle.getAngle() > maxV ? maxV
                                                        : deltaAngle.getAngle() < - maxV ? - maxV :
                                                          deltaAngle.getAngle();

        robotCommand.vel = robotToBall.rotate(- M_PI_2).stretchToLength(targetVel);
        robotCommand.angle = robotToBall.toAngle();

        return limitCommand(robotCommand);
    }
    case rotateAroundRobot: {
        int direction = targetAngle - robot->angle > 0.0 ? 1 : - 1;
        robotCommand.angle = Angle(robot->angle + maxAngularVelocity*direction);// + compensationAngle);
        robotCommand.dribbler = 1;
        return limitCommand(robotCommand);
    }
    case fastest: {
        int direction = targetAngle - robot->angle > 0.0 ? 1 : - 1;
        robotCommand.angle = Angle(robot->angle + maxAngularVelocity*direction);// + compensationAngle);
        if ((finalTargetPos - robot->pos).length2() > errorMargin*errorMargin) {
            robotCommand.vel = compensationVelocity;
        }
        robotCommand.dribbler = 1;
        return limitCommand(robotCommand);
    }
    case safest:break;
    case defaultRotate:break;
    }
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        std::cout << "rotate case not handled" << std::endl;
    }
    return {};
}

RobotCommand BallHandlePosControl::travelWithBall(TravelStrategy travelStrategy) {

    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        std::stringstream ss;
        ss << "travel with strategy: ";
        switch (travelStrategy) {
        case forwards:ss << "forwards";
            break;
        case backwards:ss << "backwards";
            break;
        case defaultTravel:ss << "default";
            break;
        }
        std::cout << ss.str() << std::endl;
    }

    RobotCommand robotCommand;
    Vector2 compensationVelocity = Vector2();
    Angle ballAngleRelativeToRobot = 0;
    if (ball->visible) {
        ballAngleRelativeToRobot = robotToBall.toAngle() - robot->angle;
    }

    switch (travelStrategy) {
    case forwards: {
        updateForwardsProgress();
    }

    case backwards: {
        if (backwardsProgress != B_overshooting && backwardsProgress != B_dribbling) {
            B_approachPosition = ball->pos + ballToRobot.stretchToLength(0.05);
        }
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

    case defaultTravel:break;
    }

    std::cout << "travel case not handled" << std::endl;
    return {
    };
}

RobotCommand BallHandlePosControl::B_startTravelBackwards() {
    B_count = 0;
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

    if (control::ControlUtils::distanceToLine(robot->pos, B_backwardsDribbleLine.first, B_backwardsDribbleLine.second)
            > errorMargin*2.0) {
        backwardsProgress = B_turning;
    }

    Angle ballAngleRelativeToRobot = Angle();
    if (ball->visible) ballAngleRelativeToRobot = robotToBall.toAngle() - robot->angle;

    command.vel += (robot->angle + M_PI_2).toVector2(ballAngleRelativeToRobot);

    return command;
}

RobotCommand BallHandlePosControl::B_sendSuccessCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.angle = B_lockedAngle;
    return command;
}

void BallHandlePosControl::updateBackwardsProgress() {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
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

    if (robotToBall.length2() > 0.5) {
        backwardsProgress = B_fail;
        return;
    }
    targetAngle = (ball->pos - finalTargetPos).toAngle();
    Angle angleDifference = robot->angle - targetAngle;

    if (backwardsProgress != B_dribbling) B_count = 0;

    switch (backwardsProgress) {
    case B_turning: {
        targetAngle = (ball->pos - finalTargetPos).toAngle();
        if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
            B_lockedAngle = (ball->pos - finalTargetPos).toAngle();
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
        if (((B_approachPosition - robot->pos)).length() < 0.06) {
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
        if (B_count > 25) {
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
        if (offsetAngle > M_PI*0.05) {
            backwardsProgress = B_start;
            return;
        }
        if ((robot->pos - finalTargetPos).length2() < errorMargin*errorMargin) {
            backwardsProgress = B_success;
            return;
        }
        return;
    }
    case B_fail:
    case B_start:
    default:return;
    case B_success: {
        if ((robot->pos - finalTargetPos).length2() < errorMargin*errorMargin) {
            return;
        }
        backwardsProgress = B_start;
    }
    }
}

void BallHandlePosControl::updateForwardsProgress() {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        std::stringstream ss;
        ss << "forwards progress:                  ";
        switch (forwardsProgress) {
        case F_start:
            ss << "start";
            break;
        case F_turning:
            ss << "turning";
            break;
        case F_approaching:
            ss << "approaching";
            break;
        case F_dribbleForward:
            ss << "dribble forwards";
            break;
        }
        std::cout << ss.str() << std::endl;
    }

}

RobotCommand BallHandlePosControl::limitCommand(RobotCommand command) {

    // velocity limiter
    double maxVel = Constants::DEFAULT_MAX_VEL();
    command.vel = control::ControlUtils::velocityLimiter(command.vel, maxVel);

    // acceleration limiter
    double maxAcc = control::ControlUtils::calculateMaxAcceleration(command.vel, command.angle);
    command.vel = control::ControlUtils::accelerationLimiter(command.vel, maxAcc, previousVelocity.length());
    previousVelocity = command.vel;
    return command;
}

RobotCommand BallHandlePosControl::goToBall(bool robotDoesNotHaveBall, bool robotIsTooFarFromBall) {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        std::cout << "we do not have a ball yet idiot" << std::endl;
    }
    RobotCommand robotCommand;

    Vector2 target = ball->pos + ballToRobot.stretchToLength(targetBallDistance);
    auto pva = numTreePosController.getPosVelAngle(robot, target);
    if (robotIsTooFarFromBall) {
        backwardsProgress = B_start;
        robotCommand.angle = pva.angle;
    }
    else {
        robotCommand.angle = robotToBall.toAngle();
    }

    robotCommand.vel = pva.vel;
    return limitCommand(robotCommand);
}

void BallHandlePosControl::updateVariables(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA) {
    double expectedDelay = 0.06;
    ball = world::world->getBall();
    robot = world::world->getFutureRobot(r, expectedDelay);
    targetPos = targetP;
    finalTargetPos = targetP;
    targetAngle = targetA;
    finalTargetAngle = targetA;
    robotToBall = ball->pos - robot->pos;
    ballToRobot = robot->pos - ball->pos;
}

} //control
} //ai
} //rtt