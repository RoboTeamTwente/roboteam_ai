//
// Created by thijs on 18-12-18.
//

#include <roboteam_ai/src/interface/api/Input.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "BallHandlePosControl.h"

namespace rtt {
namespace ai {
namespace control {

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea)
        :canMoveInDefenseArea(canMoveInDefenseArea) {
    numTreePosController.setCanMoveInDefenseArea(canMoveInDefenseArea);
    numTreePosController.setAvoidBallDistance(targetBallDistance*0.95);

    dribbleForwards = new DribbleForwards();
    dribbleBackwards = new DribbleBackwards();
}

/// targetP is the target position of the BALL, targetA is the (final) target angle of the ROBOT
RobotCommand BallHandlePosControl::getRobotCommand(const RobotPtr &r,
        const Vector2 &targetP, const Angle &targetA, TravelStrategy preferredTravelStrategy) {

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
            return rotateWithBall(rotateAroundBall);
        }
    }

    // if we do not have the ball yet, go get it
    double deltaPosSquared = (finalTargetPos - ball->pos).length2();
    bool ballIsFarFromTarget = deltaPosSquared > 0.5;

    bool robotDoesNotHaveBall = ! robot->hasBall();
    bool robotIsTooFarFromBall = ballToRobot.length2() > maxBallDistance*maxBallDistance;
    bool ballIsMovingTooFast = ball->vel.length2() > minVelForMovingball*minVelForMovingball;

    if (robotDoesNotHaveBall && (robotIsTooFarFromBall || ballIsMovingTooFast)) {
        return goToBall(ballIsFarFromTarget);
    }

    // check if we are doing something already
    if (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::B_start) return travelWithBall(backwards);
    if (dribbleForwards->getForwardsProgression() != DribbleForwards::F_start) return travelWithBall(forwards);

    // check if we are far from the final target
    if (preferredTravelStrategy != no_preference) return travelWithBall(preferredTravelStrategy);

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
        return robotCommand;
    }
    case rotateAroundRobot: {
        int direction = targetAngle - robot->angle > 0.0 ? 1 : - 1;
        robotCommand.angle = Angle(robot->angle + maxAngularVelocity*direction);
        robotCommand.dribbler = 1;
        return robotCommand;
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
        dribbleBackwards->getRobotCommand(robot, targetPos, targetAngle);
    }
    case forwards: {
        dribbleForwards->getRobotCommand(robot, targetPos, targetAngle);
    }
    case no_preference:return {};
    }

    std::cout << "travel case not handled" << std::endl;
    return {};
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

    auto pva = numTreePosController.getPosVelAngle(robot, target);
    robotCommand.angle = robotToBall.toAngle();
    robotCommand.vel = pva.vel;
    return robotCommand;
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

void BallHandlePosControl::setMaxVelocity(double maxV) {
    maxForwardsVelocity = maxV > 0.0 && maxV < 8.0 ? maxV : maxForwardsVelocity;
    maxBackwardsVelocity = maxV > 0.0 && maxV < 8.0 ? maxV : maxBackwardsVelocity;
}

BallHandlePosControl::~BallHandlePosControl() {
    delete dribbleForwards;
    delete dribbleBackwards;
}

} //control
} //ai
} //rtt