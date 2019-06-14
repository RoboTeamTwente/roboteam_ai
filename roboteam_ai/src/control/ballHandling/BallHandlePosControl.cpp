//
// Created by thijs on 18-12-18.
//

#include <roboteam_ai/src/interface/api/Input.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/world/Field.h>

#include "BallHandlePosControl.h"
#include "DribbleBackwards.h"
#include "DribbleForwards.h"
#include "RotateAroundBall.h"
#include "RotateWithBall.h"
#include "../numTrees/NumTreePosControl.h"
#include "../BasicPosControl.h"

namespace rtt {
namespace ai {
namespace control {

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea) {

    dribbleForwards = new DribbleForwards(ERROR_MARGIN, ANGLE_ERROR_MARGIN, ballPlacementAccuracy, maxForwardsVelocity);
    dribbleBackwards = new DribbleBackwards(ERROR_MARGIN, ANGLE_ERROR_MARGIN, ballPlacementAccuracy, maxBackwardsVelocity);
    rotateWithBall = new RotateWithBall();
    rotateAroundBall = new RotateAroundBall();

    setCanMoveInDefenseArea(canMoveInDefenseArea);
    setAvoidBallDistance(TARGET_BALL_DISTANCE*0.95);
}

RobotCommand BallHandlePosControl::getRobotCommand(const RobotPtr &r, const Vector2 &targetP) {
    Angle defaultAngle = lockedAngle;
    return BallHandlePosControl::getRobotCommand(r, targetP, defaultAngle);
}

RobotCommand BallHandlePosControl::getRobotCommand(const RobotPtr &r,
        const Vector2 &targetP, const Angle &targetA, TravelStrategy travelStrategy) {

    TravelStrategy tempTravelStrategy = preferredTravelStrategy;
    preferredTravelStrategy = travelStrategy;
    RobotCommand robotCommand = BallHandlePosControl::getRobotCommand(r, targetP, targetA);
    preferredTravelStrategy = tempTravelStrategy;

    return robotCommand;
}

/// targetP is the target position of the BALL, targetA is the (final) target angle of the ROBOT
RobotCommand BallHandlePosControl::getRobotCommand(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA) {

    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        printStatus();
    }

    // update PID values
    pidVals newPidValues = interface::Output::getBallHandlePid();
    updatePID(newPidValues);

    double expectedDelay = 0.04;
    ball = std::make_shared<world::Ball>(world::Ball(*world::world->getBall()));
    robot = world::world->getFutureRobot(r, expectedDelay);
    targetPos = targetP;
    targetAngle = targetA;

    // check for ball
    if (! ball) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Can't control the ball with no ball" << std::endl;
        }
        status = FAILURE;
        return {};
    }

    // if the ball is at the targetposition
    bool ballIsAtTargetPosition = (ball->pos - targetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy;
    if (ballIsAtTargetPosition) {
        return finalizeBallHandle();
    }

    lockedAngle = robot->angle;

    // if we do not have the ball yet, go get it
    bool robotDoesNotHaveBall = ! robot->hasBall();
    bool robotIsTooFarFromBall = (robot->pos - ball->pos).length2() > MAX_BALL_DISTANCE*MAX_BALL_DISTANCE;
    bool robotIsTouchingBall =
            (robot->pos - ball->pos).length() < ROBOT_IS_TOUCHING_BALL;

    bool ballIsMovingTooFast = ball->vel.length2() > MIN_VEL_FOR_MOVING_BALL*MIN_VEL_FOR_MOVING_BALL;
    bool alreadyDribbling = (
            dribbleBackwards->getBackwardsProgression() != DribbleBackwards::BackwardsProgress::START ||
                    dribbleForwards->getForwardsProgression() != DribbleForwards::ForwardsProgress::START);

    bool shouldGetBall = alreadyDribbling ?
            (ballIsMovingTooFast && ! robotIsTouchingBall) || (robotDoesNotHaveBall && robotIsTooFarFromBall) :
            (ballIsMovingTooFast && ! robotIsTouchingBall) || (robotDoesNotHaveBall || robotIsTooFarFromBall);

    bool ballIsOutsideField = ! world::field->pointIsInField(ball->pos, 0.0);
    if (ballIsOutsideField) {
        status = HANDLING_BALL;
        Vector2 targetBallPos = ControlUtils::projectPositionToWithinField(ball->pos, 1.0);
        return handleBall(targetBallPos, BACKWARDS, shouldGetBall);
    }

    bool ballIsFarFromTarget = (targetPos - ball->pos).length2() > 0.5;
    bool dribbleBackwardsWhileFarFromTarget = (preferredTravelStrategy == NO_PREFERENCE) && (ballIsFarFromTarget) &&
            (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::BackwardsProgress::START);

    if (dribbleBackwardsWhileFarFromTarget) {
        dribbleBackwards->reset();
    }
    return handleBall(targetPos, preferredTravelStrategy, shouldGetBall, ballIsFarFromTarget);
}

RobotCommand BallHandlePosControl::finalizeBallHandle() {
    dribbleBackwards->reset();
    dribbleForwards->reset();
    if (robot->getDribblerState() > 0 || ! robot->isDribblerReady()) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Waiting for the dribbler to stop" << std::endl;
        }
        RobotCommand robotCommand;
        robotCommand.vel = {0, 0};
        robotCommand.angle = lockedAngle;
        robotCommand.dribbler = 0;
        status = FINALIZING;
        return controlWithPID(xBallHandlePID, yBallHandlePID, robotCommand);
    }
    else if (fabs(lockedAngle - robot->angle) > ANGLE_ERROR_MARGIN) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Rotating robot to final angle" << std::endl;
        }
        status = FINALIZING;
        return controlWithPID(xBallHandlePID, yBallHandlePID,
                rotateAroundBall->getRobotCommand(robot, targetPos, lockedAngle));
    }
    else {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Success!!" << std::endl;
        }
        RobotCommand robotCommand;
        robotCommand.vel = {0, 0};
        robotCommand.angle = lockedAngle;
        robotCommand.dribbler = 0;
        status = SUCCESS;
        return robotCommand;
    }
}

void BallHandlePosControl::printStatus() {
    switch (status) {
    case GET_BALL:std::cout << "status: GET_BALL" << std::endl;
        return;
    case HANDLING_BALL:std::cout << "status: HANDLING_BALL" << std::endl;
        return;
    case FINALIZING:std::cout << "status: FINALIZING" << std::endl;
        return;
    case SUCCESS:std::cout << "status: SUCCESS" << std::endl;
        return;
    default:
    case FAILURE:std::cout << "status: FAILURE" << std::endl;
        return;
    }
}

BallHandlePosControl::~BallHandlePosControl() {
    delete dribbleForwards;
    delete dribbleBackwards;
    delete rotateAroundBall;
    delete rotateWithBall;
}

void BallHandlePosControl::setMaxVelocity(double maxV) {
    setMaxForwardsVelocity(maxV);
    setMaxBackwardsVelocity(maxV);
}

void BallHandlePosControl::setMaxForwardsVelocity(double maxV) {
    if (maxV < 0) {
        std::cout << "Setting invalid max velocity in BallHandlePosControl" << std::endl;
        return;
    }
    if (maxV > RefGameState().getRuleSet().maxRobotVel) {
        maxV = RefGameState().getRuleSet().maxRobotVel;
    }
    maxForwardsVelocity = maxV;
    dribbleForwards->setMaxVel(maxForwardsVelocity);
}

void BallHandlePosControl::setMaxBackwardsVelocity(double maxV) {
    if (maxV < 0) {
        std::cout << "Setting invalid max velocity in BallHandlePosControl" << std::endl;
        return;
    }
    if (maxV > RefGameState().getRuleSet().maxRobotVel) {
        maxV = RefGameState().getRuleSet().maxRobotVel;
    }
    maxBackwardsVelocity = maxV;
    dribbleBackwards->setMaxVel(maxBackwardsVelocity);

}

BallHandlePosControl::Status BallHandlePosControl::getStatus() {
    return status;
}

RobotCommand BallHandlePosControl::handleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy,
        bool shouldGoToBall, bool ballIsFarFromTarget) {

    if (shouldGoToBall) {
        status = GET_BALL;
        return goToBall(targetBallPos, travelStrategy, ballIsFarFromTarget);
    }

    status = HANDLING_BALL;
    // check if we are doing something already
    if (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::START) {
        return controlWithPID(xBallHandlePID, yBallHandlePID,
                dribbleBackwards->getRobotCommand(robot, targetBallPos, targetAngle));
    }

    if (dribbleForwards->getForwardsProgression() != DribbleForwards::START) {
        return controlWithPID(xBallHandlePID, yBallHandlePID,
                dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle));
    }

    switch (travelStrategy) {
    case FORWARDS:
        return controlWithPID(xBallHandlePID, yBallHandlePID,
                dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle));

    case BACKWARDS:
        return controlWithPID(xBallHandlePID, yBallHandlePID,
                dribbleBackwards->getRobotCommand(robot, targetBallPos, targetAngle));

    default:
    case NO_PREFERENCE: {
        // choose based on distance from the ball to the target
        if (ballIsFarFromTarget) {
            return controlWithPID(xBallHandlePID, yBallHandlePID,
                    dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle));
        }
        return controlWithPID(xBallHandlePID, yBallHandlePID,
                dribbleBackwards->getRobotCommand(robot, targetBallPos, targetAngle));
    }
    }

}

RobotCommand BallHandlePosControl::goToBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy,
        bool ballIsFarFromTarget) {

    if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
        std::cout << "we do not have a ball yet" << std::endl;
    }

    dribbleBackwards->reset();
    dribbleForwards->reset();

    bool ballIsMoving = ball->vel.length2() > MIN_VEL_FOR_MOVING_BALL*MIN_VEL_FOR_MOVING_BALL;

    if (ballIsMoving) {
        return controlWithPID(xGoToBallPID, yGoToBallPID,
                goToMovingBall());
    }
    return controlWithPID(xGoToBallPID, yGoToBallPID,
            goToIdleBall(targetBallPos, travelStrategy, ballIsFarFromTarget));
}

RobotCommand BallHandlePosControl::goToMovingBall() {
    Vector2 numTreesTarget;

    Vector2 ballStillPosition = ball->getBallStillPosition();

    LineSegment ballLine = LineSegment(ball->pos, ball->pos + ball->vel);
    Vector2 projectionPosition = ballLine.project(robot->pos);
    double robotToProjectionDistance = (projectionPosition - robot->pos).length();
    double ballToProjectionDistance = (projectionPosition - ball->pos).length();
    const double AVERAGE_ROBOT_INTERCEPT_VELOCITY = 1.41; // ms-1

    Angle robotAngleTowardsBallVel = (robot->pos - ball->pos).toAngle() - ball->vel.toAngle();
    bool robotIsBehindBall = fabs(robotAngleTowardsBallVel) < M_PI_4;

    bool robotCanInterceptBall = robotToProjectionDistance/AVERAGE_ROBOT_INTERCEPT_VELOCITY <
            ballToProjectionDistance/ball->vel.length();

    if (robotIsBehindBall && robotCanInterceptBall) {
        numTreesTarget = projectionPosition;
        auto robotCommand = NumTreePosControl::getRobotCommand(robot, numTreesTarget);

        robotCommand.angle = (ball->pos - robot->pos).toAngle();
        if (fabs(robotAngleTowardsBallVel) > M_PI*0.05) {
            robotCommand.vel += ball->vel.stretchToLength(
                    fabs((robot->pos - ball->pos).toAngle() - ball->vel.toAngle()));
        }
        else if (ballToProjectionDistance/ball->vel.length() > 0.8) {
            robotCommand.vel -= ball->vel;
        }

        return robotCommand;
    }

    numTreesTarget = ControlUtils::projectPositionToWithinField(ballStillPosition);
    auto robotCommand = NumTreePosControl::getRobotCommand(robot, numTreesTarget);

    if (NumTreePosControl::getCurrentCollisionWithRobot().getCollisionType() == Collision::CollisionType::BALL) {
        robotCommand.vel = (ball->pos - robot->pos).stretchToLength(ball->vel.length());
    }

    robotCommand.vel += ball->vel;
    return robotCommand;
}

RobotCommand BallHandlePosControl::goToIdleBall(
        const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget) {
    Vector2 numTreesTarget;

    Vector2 ballToTarget = targetBallPos - ball->pos;
    if (travelStrategy == BACKWARDS) {
        numTreesTarget = ball->pos + ballToTarget.stretchToLength(MAX_BALL_DISTANCE);
    }
    else if (travelStrategy == FORWARDS) {
        numTreesTarget = ball->pos + ballToTarget.stretchToLength(- MAX_BALL_DISTANCE);
    }
    else if (ballIsFarFromTarget) {
        numTreesTarget = ball->pos + ballToTarget.stretchToLength(- MAX_BALL_DISTANCE);
    }
    else {
        numTreesTarget = ball->pos + ballToTarget.stretchToLength(MAX_BALL_DISTANCE);
    }
    auto robotCommand = NumTreePosControl::getRobotCommand(robot, numTreesTarget);
    if ((ball->pos - robot->pos).length() < MAX_BALL_DISTANCE*1.5) {
        robotCommand.angle = (ball->pos - robot->pos).toAngle();
    }
    return robotCommand;
}

RobotCommand BallHandlePosControl::controlWithPID(PID &xpid, PID &ypid, const RobotCommand &robotCommand) {
    RobotCommand pidCommand = robotCommand;
    pidCommand.vel.x = xpid.getOutput(robot->vel.x, robotCommand.vel.x);
    pidCommand.vel.y = ypid.getOutput(robot->vel.y, robotCommand.vel.y);
    return pidCommand;
}

void BallHandlePosControl::updatePID(pidVals newPID) {
    if (newPID != lastPid) {
        lastPid = newPID;
        xBallHandlePID.setPID(newPID);
        yBallHandlePID.setPID(newPID);
    }
}

} //control
} //ai
} //rtt