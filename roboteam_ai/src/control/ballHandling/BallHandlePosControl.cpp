//
// Created by thijs on 18-12-18.
//

#include <roboteam_ai/src/interface/api/Input.h>
#include <roboteam_ai/src/control/ControlUtils.h>

#include "BallHandlePosControl.h"
#include "DribbleBackwards.h"
#include "DribbleForwards.h"
#include "RotateAroundBall.h"
#include "RotateWithBall.h"
#include "../numTrees/NumTreePosControl.h"

namespace rtt {
namespace ai {
namespace control {

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea)
        :canMoveInDefenseArea(canMoveInDefenseArea) {

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

    if (true) {
        printStatus();
    }

    double expectedDelay = 0.04;
    ball = world::world->getBall();
    robot = world::world->getFutureRobot(r, expectedDelay);
    targetPos = targetP;
    finalTargetPos = targetP;
    targetAngle = targetA;
    finalTargetAngle = targetA;

    // check for ball
    if (! ball) {
        if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Can't control the ball with no ball" << std::endl;
        }
        status = FAILURE;
        return {};
    }

    // if the ball is at the targetposition
    if ((ball->pos - finalTargetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy) {
        dribbleBackwards->reset();
        dribbleForwards->reset();
        if (robot->getDribblerState() > 0 || ! robot->isDribblerReady()) {
            if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
                std::cout << "Waiting for the dribbler to stop" << std::endl;
            }
            RobotCommand robotCommand;
            robotCommand.vel = {0, 0};
            robotCommand.angle = lockedAngle;
            robotCommand.dribbler = 0;
            status = FINALIZING;
            return robotCommand;
        }
        else if (fabs(lockedAngle - robot->angle) > angleErrorMargin) {
            if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
                std::cout << "Rotating robot to final angle" << std::endl;
            }
            status = FINALIZING;
            return rotateAroundBall->getRobotCommand(robot, targetPos, lockedAngle);
        }
        else {
            if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
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
    else {
        lockedAngle = robot->angle;
    }

    // if we do not have the ball yet, go get it
    double deltaPosSquared = (finalTargetPos - ball->pos).length2();
    bool ballIsFarFromTarget = deltaPosSquared > 0.5;

    bool robotDoesNotHaveBall = ! robot->hasBall();
    bool robotIsTooFarFromBall = (robot->pos - ball->pos).length2() > maxBallDistance*maxBallDistance;
    bool ballIsMovingTooFast = ball->vel.length2() > minVelForMovingball*minVelForMovingball;

    if (robotDoesNotHaveBall && (robotIsTooFarFromBall || ballIsMovingTooFast)) {
        status = GET_BALL;
        return goToBall(ballIsFarFromTarget);
    }

    status = HANDLING_BALL;
    // check if we are doing something already
    if (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::START) {
        return dribbleBackwards->getRobotCommand(robot, targetPos, targetAngle);
    }

    if (dribbleForwards->getForwardsProgression() != DribbleForwards::START) {
        return dribbleForwards->getRobotCommand(robot, targetPos, targetAngle);
    }

    switch (preferredTravelStrategy) {
    case FORWARDS: return dribbleForwards->getRobotCommand(robot, targetPos, targetAngle);
    case BACKWARDS: return dribbleBackwards->getRobotCommand(robot, targetPos, targetAngle);
    default:
    case NO_PREFERENCE: {
        // choose based on distance from the ball to the target
        if (ballIsFarFromTarget) {
            return dribbleForwards->getRobotCommand(robot, targetPos, targetAngle);
        }
        return dribbleBackwards->getRobotCommand(robot, targetPos, targetAngle);
    }
    }

}

void BallHandlePosControl::printStatus() {
    switch (status) {
    case GET_BALL:std::cout << "status: GET_BALL" << std::endl;return;
    case HANDLING_BALL:std::cout << "status: HANDLING_BALL" << std::endl;return;
    case FINALIZING:std::cout << "status: FINALIZING" << std::endl;return;
    case SUCCESS:std::cout << "status: SUCCESS" << std::endl;return;
    default:
    case FAILURE:std::cout << "status: FAILURE" << std::endl;return;
    }
}

RobotCommand BallHandlePosControl::goToBall(bool ballIsFarFromTarget) {

    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        std::cout << "we do not have a ball yet" << std::endl;
    }

    if (ball->vel.length2() > minVelForMovingball*minVelForMovingball) {
        ball->pos += (ball->vel*0.5 + ball->vel.stretchToLength(pow(ball->vel.length2(), 1.0/8.0)));
    }

    dribbleBackwards->reset();
    dribbleForwards->reset();
    RobotCommand robotCommand;
    Vector2 target;
    Vector2 ballToTarget = finalTargetPos - ball->pos;
    if (preferredTravelStrategy == BACKWARDS) {
        target = ball->pos + ballToTarget.stretchToLength(maxBallDistance);
    }
    else if (preferredTravelStrategy == FORWARDS) {
        target = ball->pos + ballToTarget.stretchToLength(- maxBallDistance);
    }
    else if (ballIsFarFromTarget) {
        target = ball->pos + ballToTarget.stretchToLength(- maxBallDistance);
    }
    else {
        target = ball->pos + ballToTarget.stretchToLength(maxBallDistance);
    }

    auto path = NumTreePosControl::getRobotCommand(robot, target);
    robotCommand.angle = (ball->pos - robot->pos).toAngle();
    robotCommand.vel = path.vel;
    return robotCommand;
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

} //control
} //ai
} //rtt