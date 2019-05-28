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
    numTreePosControl = new NumTreePosControl();

    numTreePosControl->setCanMoveInDefenseArea(canMoveInDefenseArea);
    numTreePosControl->setAvoidBallDistance(targetBallDistance*0.95);
}

/// targetP is the target position of the BALL, targetA is the (final) target angle of the ROBOT
RobotCommand BallHandlePosControl::getRobotCommand(const RobotPtr &r,
        const Vector2 &targetP, const Angle &targetA, TravelStrategy preferredTravelStrategy) {

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
            return robotCommand;
        }
        else {
            if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
                std::cout << "Rotating robot to final angle" << std::endl;
            }
            return rotateAroundBall->getRobotCommand(robot, targetPos, targetAngle);
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
        return goToBall(ballIsFarFromTarget);
    }

    // check if we are doing something already
    if (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::START) {
        return dribbleBackwards->getRobotCommand(robot, targetPos, targetAngle);
    }

    if (dribbleForwards->getForwardsProgression() != DribbleForwards::START) {
        return dribbleForwards->getRobotCommand(robot, targetPos, targetAngle);
    }

    switch (preferredTravelStrategy) {
    case forwards: return dribbleForwards->getRobotCommand(robot, targetPos, targetAngle);
    case backwards: return dribbleBackwards->getRobotCommand(robot, targetPos, targetAngle);
    default:
    case no_preference: {
        // choose based on distance from the ball to the target
        if (ballIsFarFromTarget) {
            return dribbleForwards->getRobotCommand(robot, targetPos, targetAngle);
        }
        return dribbleBackwards->getRobotCommand(robot, targetPos, targetAngle);
    }
    }

}

RobotCommand BallHandlePosControl::goToBall(bool ballIsFarFromTarget, TravelStrategy preferredTravelStrategy) {

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
    if (preferredTravelStrategy == backwards) {
        target = ball->pos + ballToTarget.stretchToLength(maxBallDistance);
    }
    else if (preferredTravelStrategy == forwards) {
        target = ball->pos + ballToTarget.stretchToLength(- maxBallDistance);
    }
    else if (ballIsFarFromTarget) {
        target = ball->pos + ballToTarget.stretchToLength(- maxBallDistance);
    }
    else {
        target = ball->pos + ballToTarget.stretchToLength(maxBallDistance);
    }

    auto pva = numTreePosControl->getPosVelAngle(robot, target);
    robotCommand.angle = (ball->pos - robot->pos).toAngle();
    robotCommand.vel = pva.vel;
    return robotCommand;
}

BallHandlePosControl::~BallHandlePosControl() {
    delete dribbleForwards;
    delete dribbleBackwards;
    delete rotateAroundBall;
    delete rotateWithBall;
    delete numTreePosControl;
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

} //control
} //ai
} //rtt