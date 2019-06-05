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
#include "../positionControllers/BasicPosControl.h"

namespace rtt {
namespace ai {
namespace control {

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea) {

    dribbleForwards = new DribbleForwards(errorMargin, angleErrorMargin, ballPlacementAccuracy, maxForwardsVelocity);
    dribbleBackwards = new DribbleBackwards(errorMargin, angleErrorMargin, ballPlacementAccuracy, maxBackwardsVelocity);
    rotateWithBall = new RotateWithBall();
    rotateAroundBall = new RotateAroundBall();

    setCanMoveInDefenseArea(canMoveInDefenseArea);
    setAvoidBallDistance(targetBallDistance*0.95);
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
    bool robotIsTooFarFromBall = (robot->pos - ball->pos).length2() > maxBallDistance*maxBallDistance;
    bool ballIsMovingTooFast = ball->vel.length2() > minVelForMovingball*minVelForMovingball;
    bool shouldGetBall = robotDoesNotHaveBall && (robotIsTooFarFromBall || ballIsMovingTooFast);

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
            cout << "Waiting for the dribbler to stop" << endl;
        }
        RobotCommand robotCommand;
        robotCommand.vel = {0, 0};
        robotCommand.angle = lockedAngle;
        robotCommand.dribbler = 0;
        status = FINALIZING;
        return robotCommand;
    }
    else if (fabs(lockedAngle - robot->angle) > angleErrorMargin) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            cout << "Rotating robot to final angle" << endl;
        }
        status = FINALIZING;
        return rotateAroundBall->getRobotCommand(robot, targetPos, lockedAngle);
    }
    else {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            cout << "Success!!" << endl;
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
        return dribbleBackwards->getRobotCommand(robot, targetBallPos, targetAngle);
    }

    if (dribbleForwards->getForwardsProgression() != DribbleForwards::START) {
        return dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle);
    }

    switch (travelStrategy) {
    case FORWARDS: return dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle);
    case BACKWARDS: return dribbleBackwards->getRobotCommand(robot, targetBallPos, targetAngle);
    default:
    case NO_PREFERENCE: {
        // choose based on distance from the ball to the target
        if (ballIsFarFromTarget) {
            return dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle);
        }
        return dribbleBackwards->getRobotCommand(robot, targetBallPos, targetAngle);
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

    bool ballIsMoving = ball->vel.length2() > minVelForMovingball*minVelForMovingball;

    if (ballIsMoving) {
        return goToMovingBall();
    }
    return goToIdleBall(targetBallPos, travelStrategy, ballIsFarFromTarget);
}

RobotCommand BallHandlePosControl::goToMovingBall() {
    Vector2 numTreesTarget;

    Vector2 ballStillPosition = ball->getBallStillPosition();

    LineSegment ballLine = LineSegment(ball->pos, ball->pos + ball->vel);
    Vector2 projectionPosition = ballLine.project(robot->pos);
    double robotToProjectionDistance = (projectionPosition - robot->pos).length();
    double ballToProjectionDistance = (projectionPosition - ball->pos).length();
    const double interceptionConstant = 2.0;

    Angle robotAngleTowardsBallVel = (robot->pos - ball->pos).toAngle() - ball->vel.toAngle();
    bool robotIsBehindBall = fabs(robotAngleTowardsBallVel) < M_PI_4;

    bool robotCanInterceptBall = robotToProjectionDistance <
            interceptionConstant*ballToProjectionDistance/ball->vel.length();

    if (robotIsBehindBall && robotCanInterceptBall) {
        numTreesTarget = projectionPosition;
        auto robotCommand = NumTreePosControl::getRobotCommand(robot, numTreesTarget);

        robotCommand.angle = (ball->pos - robot->pos).toAngle();
        if (fabs(robotAngleTowardsBallVel) > 0.1) {
            robotCommand.vel += ball->vel.stretchToLength(
                    fabs((robot->pos - ball->pos).toAngle() - ball->vel.toAngle()));
        }
        return robotCommand;
    }

    numTreesTarget = ballStillPosition;
    auto robotCommand = NumTreePosControl::getRobotCommand(robot, numTreesTarget);

    robotCommand.vel += std::max(ball->vel, ball->vel.normalize());
    return robotCommand;
}

RobotCommand BallHandlePosControl::goToIdleBall(
        const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget) {
    Vector2 numTreesTarget;

    Vector2 ballToTarget = targetBallPos - ball->pos;
    if (travelStrategy == BACKWARDS) {
        numTreesTarget = ball->pos + ballToTarget.stretchToLength(maxBallDistance);
    }
    else if (travelStrategy == FORWARDS) {
        numTreesTarget = ball->pos + ballToTarget.stretchToLength(- maxBallDistance);
    }
    else if (ballIsFarFromTarget) {
        numTreesTarget = ball->pos + ballToTarget.stretchToLength(- maxBallDistance);
    }
    else {
        numTreesTarget = ball->pos + ballToTarget.stretchToLength(maxBallDistance);
    }
    return NumTreePosControl::getRobotCommand(robot, numTreesTarget);
}

} //control
} //ai
} //rtt