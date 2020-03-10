//
// Created by thijs on 18-12-18.
//

#include "control/ball-handling/BallHandlePosControl.h"
#include <control/ControlUtils.h>
#include <interface/api/Input.h>
#include <world/FieldComputations.h>
#include "control/BasicPosControl.h"
#include "control/ball-handling/DribbleBackwards.h"
#include "control/ball-handling/DribbleForwards.h"
#include "control/ball-handling/RotateAroundBall.h"
#include "control/ball-handling/RotateWithBall.h"
#include "control/numtrees/NumTreePosControl.h"
#include "world_new/World.hpp"

namespace rtt::ai::control {

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea) {
    dribbleForwards = new DribbleForwards(ERROR_MARGIN, ANGLE_ERROR_MARGIN, ballPlacementAccuracy, maxForwardsVelocity);
    dribbleBackwards = new DribbleBackwards(ERROR_MARGIN, ANGLE_ERROR_MARGIN, ballPlacementAccuracy, maxBackwardsVelocity);
    rotateWithBall = new RotateWithBall();
    rotateAroundBall = new RotateAroundBall();

    setCanMoveInDefenseArea(canMoveInDefenseArea);
    setAvoidBallDistance(MAX_BALL_DISTANCE * 0.92);
}

// TODO: Implement this function
RobotCommand BallHandlePosControl::getRobotCommand(int robotId, const Vector2 &targetP, const Angle &targetA, TravelStrategy travelStrategy) {
    TravelStrategy tempTravelStrategy = preferredTravelStrategy;
    preferredTravelStrategy = travelStrategy;
    RobotCommand robotCommand = BallHandlePosControl::getRobotCommand(robotId, targetP, targetA);
    preferredTravelStrategy = tempTravelStrategy;

    return robotCommand;
}

// TODO: Implement this function
RobotCommand BallHandlePosControl::getRobotCommand(int robotId, const Vector2 &targetP, const Angle &targetA) {
    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        printStatus();
    }

    // update PID values
    pidVals newPidValues = interface::Output::getBallHandlePid();
    updatePID(newPidValues);

    double expectedDelay = 0.04;
    auto _ball = world_new::World::instance()->getWorld()->getBall();
    auto _robot = world_new::World::instance()->getWorld()->getRobotForId(robotId, true);

    if ((targetPos - targetP).length2() > 0.10) {
        dribbleBackwards->reset();
        dribbleForwards->reset();
    }
    targetPos = targetP;
    targetAngle = targetA;

    // check for ball
    if (!_ball) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Can't control the ball with no ball" << std::endl;
        }
        status = FAILURE;
        return {};
    }

    // if the ball is at the targetposition
    bool ballIsAtTargetPosition = (_ball->get()->getPos() - targetPos).length2() < ballPlacementAccuracy * ballPlacementAccuracy;
    if (ballIsAtTargetPosition) {
        return finalizeBallHandle(_robot.value());
    }

    lockedAngle = _robot->get()->getAngle();

    // if we do not have the ball yet, go get it
    bool robotDoesNotHaveBall = !_robot->hasBall();
    bool robotIsTooFarFromBall = (_robot->get()->getPos() - _ball->get()->getPos()).length2() > MAX_BALL_DISTANCE * MAX_BALL_DISTANCE;
    bool robotIsTouchingBall = (_robot->get()->getPos() - _ball->get()->getPos()).length() < ROBOT_IS_TOUCHING_BALL;

    bool ballIsMovingTooFast = _ball->get()->getVelocity().length2() > MIN_VEL_FOR_MOVING_BALL * MIN_VEL_FOR_MOVING_BALL;
    bool alreadyDribbling = (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::BackwardsProgress::START ||
                             dribbleForwards->getForwardsProgression() != DribbleForwards::ForwardsProgress::START);

    bool shouldGetBall = alreadyDribbling ? (ballIsMovingTooFast && !robotIsTouchingBall) || (robotDoesNotHaveBall && robotIsTooFarFromBall)
                                          : (ballIsMovingTooFast && !robotIsTouchingBall) || (robotDoesNotHaveBall || robotIsTooFarFromBall);

    bool ballIsOutsideField = !FieldComputations::pointIsInField(*world_new::World::instance()->getField(), _ball->get()->getPos(), 0.0);
    if (ballIsOutsideField) {
        status = HANDLING_BALL;
        Vector2 targetBallPos = ControlUtils::projectPositionToWithinField(*world_new::World::instance()->getField(), _ball->get()->getPos(), 1.0);
        return handleBall(targetBallPos, BACKWARDS, shouldGetBall, _robot.value());
    }

    bool ballIsFarFromTarget = (targetPos - _ball->get()->getPos()).length2() > 0.5;
    bool dribbleBackwardsWhileFarFromTarget =
        (preferredTravelStrategy == NO_PREFERENCE) && (ballIsFarFromTarget) && (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::BackwardsProgress::START);

    if (dribbleBackwardsWhileFarFromTarget) {
        dribbleBackwards->reset();
    }
    return handleBall(targetPos, preferredTravelStrategy, shouldGetBall, _robot.value(), ballIsFarFromTarget);
}

RobotCommand BallHandlePosControl::finalizeBallHandle(world_new::view::RobotView _robot) {
    dribbleBackwards->reset();
    dribbleForwards->reset();

    if (_robot->getDribblerState() > 0) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Waiting for the dribbler to stop" << std::endl;
        }
        RobotCommand robotCommand;
        robotCommand.vel = {0, 0};
        robotCommand.angle = lockedAngle;
        robotCommand.dribbler = _robot->getVel().length() < 0.15 ? 0 : 27;
        status = FINALIZING;
        return controlWithPID(xBallHandlePID, yBallHandlePID, robotCommand, _robot);
    } else if (fabs(lockedAngle - _robot->getAngle()) > ANGLE_ERROR_MARGIN) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Rotating robot to final angle" << std::endl;
        }
        status = FINALIZING;
        return controlWithPID(xBallHandlePID, yBallHandlePID, rotateAroundBall->getRobotCommand(_robot, targetPos, lockedAngle), _robot);
    } else {
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
        case GET_BALL:
            std::cout << "status: GET_BALL" << std::endl;
            return;
        case HANDLING_BALL:
            std::cout << "status: HANDLING_BALL" << std::endl;
            return;
        case FINALIZING:
            std::cout << "status: FINALIZING" << std::endl;
            return;
        case SUCCESS:
            std::cout << "status: SUCCESS" << std::endl;
            return;
        default:
        case FAILURE:
            std::cout << "status: FAILURE" << std::endl;
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

BallHandlePosControl::Status BallHandlePosControl::getStatus() { return status; }

RobotCommand BallHandlePosControl::handleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool shouldGoToBall, world_new::view::RobotView _robot,
                                              bool ballIsFarFromTarget) {
    if (shouldGoToBall) {
        status = GET_BALL;
        return goToBall(targetBallPos, travelStrategy, ballIsFarFromTarget, _robot);
    }

    status = HANDLING_BALL;
    // check if we are doing something already
    if (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::START) {
        return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleBackwards->getRobotCommand(_robot, targetBallPos, targetAngle), _robot);
    }

    if (dribbleForwards->getForwardsProgression() != DribbleForwards::START) {
        return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleForwards->getRobotCommand(_robot, targetBallPos, targetAngle), _robot);
    }

    switch (travelStrategy) {
        case FORWARDS:
            return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleForwards->getRobotCommand(_robot, targetBallPos, targetAngle), _robot);

        case BACKWARDS:
            return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleBackwards->getRobotCommand(_robot, targetBallPos, targetAngle), _robot);

        default:
        case NO_PREFERENCE: {
            // choose based on distance from the ball to the target
            if (ballIsFarFromTarget) {
                return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleForwards->getRobotCommand(_robot, targetBallPos, targetAngle), _robot);
            }
            return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleBackwards->getRobotCommand(_robot, targetBallPos, targetAngle), _robot);
        }
    }
}

RobotCommand BallHandlePosControl::goToBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget, world_new::view::RobotView _robot) {
    if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
        std::cout << "we do not have a ball yet" << std::endl;
    }

    dribbleBackwards->reset();
    dribbleForwards->reset();

    bool ballIsMoving = world_new::World::instance()->getWorld()->getBall().value()->getVelocity().length2() > MIN_VEL_FOR_MOVING_BALL * MIN_VEL_FOR_MOVING_BALL;

    if (ballIsMoving) {
        return controlWithPID(xGoToBallPID, yGoToBallPID, goToMovingBall(_robot), _robot);
    }
    return controlWithPID(xGoToBallPID, yGoToBallPID, goToIdleBall(targetBallPos, travelStrategy, ballIsFarFromTarget, _robot), _robot);
}

RobotCommand BallHandlePosControl::goToMovingBall(world_new::view::RobotView _robot) {
    auto _ball = world_new::World::instance()->getWorld()->getBall().value();
    Vector2 ballStillPosition = _ball->getExpectedEndPosition();

    LineSegment ballLine = LineSegment(_ball->getPos(), ballStillPosition);
    Vector2 projectionPosition = ballLine.project(_robot->getPos());
    double robotToProjectionDistance = (projectionPosition - _robot->getPos()).length();
    double ballToProjectionDistance = (projectionPosition - _ball->getPos()).length();
    const double AVERAGE_ROBOT_INTERCEPT_VELOCITY = 1.41;  // ms-1

    Angle robotAngleTowardsBallVel = (_robot->getPos() - _ball->getPos()).toAngle() - _ball->getVelocity().toAngle();
    bool robotIsBehindBall = fabs(robotAngleTowardsBallVel) < M_PI_4;

    bool robotCanInterceptBall = robotToProjectionDistance / AVERAGE_ROBOT_INTERCEPT_VELOCITY < ballToProjectionDistance / _ball->getVelocity().length();

    RobotCommand robotCommand;
    if (robotIsBehindBall && robotCanInterceptBall) {
        robotCommand = interceptMovingBall(projectionPosition, ballToProjectionDistance, robotAngleTowardsBallVel, _robot);
    } else if (!robotIsBehindBall && false) {  // TODO::do not use this till interceptMovingBallTowardsBall() works
        robotCommand = interceptMovingBallTowardsBall(_robot);
    } else {
        robotCommand = goBehindBall(ballStillPosition, _robot);
    }
    if ((_robot->getPos() - _ball->getPos()).length() < 0.5) {
        robotCommand.dribbler = 31;
    }
    return robotCommand;
}

RobotCommand BallHandlePosControl::goBehindBall(const Vector2 &ballStillPosition, world_new::view::RobotView _robot) {
    Vector2 numTreesTarget = ballStillPosition;
    auto _field = world_new::World::instance()->getField();
    auto _ball = world_new::World::instance()->getWorld()->getBall().value();

    if (!FieldComputations::pointIsInField(*_field, ballStillPosition, Constants::ROBOT_RADIUS())) {
        LineSegment ballLine = LineSegment(_ball->getPos(), ballStillPosition);
        Polygon fieldEdge = FieldComputations::getFieldEdge(*_field, Constants::ROBOT_RADIUS());

        auto intersections = fieldEdge.intersections(ballLine);
        if (intersections.size() == 1) {
            numTreesTarget = intersections[0];
        } else {
            numTreesTarget = ControlUtils::projectPositionToWithinField(*_field, ballStillPosition);
        }
    }

    auto robotCommand = NumTreePosControl::getRobotCommand(_robot->getId(), numTreesTarget);

    if (getCurrentCollisionWithRobot().getCollisionType() == Collision::BALL) {
        robotCommand.vel = (_ball->getPos() - _robot->getPos()).stretchToLength(_ball->getVelocity().length());
    }

    Vector2 targetVelIncrease = _ball->getVelocity();
    LineSegment driveLine = {_robot->getPos(), _robot->getPos() + (robotCommand.vel + targetVelIncrease).stretchToLength(_robot->getVel().length() * 2.0)};
    if (!isCrashingIntoOpponentRobot(driveLine, _robot) && !isCrashingOutsideField(driveLine, _robot)) {
        robotCommand.vel += targetVelIncrease;
    }

    return robotCommand;
}

RobotCommand BallHandlePosControl::interceptMovingBallTowardsBall(world_new::view::RobotView _robot) {
    auto field = world_new::World::instance()->getField();
    auto ball = world_new::World::instance()->getWorld()->getBall().value();
    Angle robotAngleTowardsBall = ball->getVelocity().toAngle() - (ball->getPos() - _robot->getPos()).toAngle();

    if (fabs(robotAngleTowardsBall) < M_PI * 0.12) {
        LineSegment ntLine = LineSegment(ball->getPos(), ball->getExpectedEndPosition());

        if (ntLine.distanceToLine(movingBallTowardsBallTarget) > 0.3) {
            movingBallTowardsBallTarget = ball->getPos() + (ball->getVelocity()).stretchToLength(std::min(0.5, (ball->getExpectedEndPosition() - ball->getPos()).length()));
        }
    } else {
        Line ntLine = Line(ball->getPos(), ball->getExpectedEndPosition());
        Vector2 projection = ntLine.project(_robot->getPos());

        movingBallTowardsBallTarget = ball->getPos() / 2 + projection / 2;
    }

    if (!FieldComputations::pointIsInField(*field, movingBallTowardsBallTarget, Constants::ROBOT_RADIUS())) {
        LineSegment ballLine = LineSegment(ball->getPos(), movingBallTowardsBallTarget);
        Polygon fieldEdge = FieldComputations::getFieldEdge(*field, Constants::ROBOT_RADIUS());

        auto intersections = fieldEdge.intersections(ballLine);
        if (intersections.size() == 1) {
            movingBallTowardsBallTarget = intersections[0];
        } else {
            movingBallTowardsBallTarget = ControlUtils::projectPositionToWithinField(*field, ball->getExpectedEndPosition());
        }
    }

    auto tempAvoidBallDistance = getAvoidBallDistance();
    setAvoidBallDistance(0.0);

    auto robotCommand = NumTreePosControl::getRobotCommand(_robot->getId(), movingBallTowardsBallTarget);

    setAvoidBallDistance(tempAvoidBallDistance);

    if (FieldComputations::pointIsInField(*field, _robot->getPos() + _robot->getVel())) {
        double ballVel = ball->getVelocity().length();
        double targetVel = ballVel * 2.4;

        double distanceToBall = (_robot->getPos() - ball->getPos()).length();
        if (distanceToBall < 1.0) {
            targetVel = ballVel * (1.4 + distanceToBall);
        }

        double commandVel = robotCommand.vel.length();
        targetVel = std::max(targetVel, commandVel);

        Vector2 targetVelIncrease = ball->getVelocity().stretchToLength(targetVel / 2);
        LineSegment driveLine = {_robot->getPos(), _robot->getPos() + targetVelIncrease.stretchToLength(_robot->getVel().length() * 2.0)};
        if (!isCrashingIntoOpponentRobot(driveLine, _robot) && !isCrashingOutsideField(driveLine, _robot)) {
            robotCommand.vel += targetVelIncrease;
        }
    }
    if ((_robot->getPos() - ball->getPos()).length() < 0.5) {
        robotCommand.dribbler = 31;
    }
    return robotCommand;
}

RobotCommand BallHandlePosControl::interceptMovingBall(const Vector2 &projectionPosition, double ballToProjectionDistance, const Angle &robotAngleTowardsBallVel,
                                                       world_new::view::RobotView _robot) {
    Vector2 numTreesTarget = projectionPosition;
    RobotCommand robotCommand;

    LineSegment driveLine = LineSegment(_robot->getPos(), projectionPosition.stretchToLength(_robot->getVel().length() * 2.0));
    if (!isCrashingIntoOpponentRobot(driveLine, _robot) && !isCrashingOutsideField(driveLine, _robot)) {
        robotCommand = BasicPosControl::getRobotCommand(_robot->getId(), numTreesTarget);
    } else {
        robotCommand = NumTreePosControl::getRobotCommand(_robot->getId(), numTreesTarget);
    }

    auto _ball = world_new::World::instance()->getWorld()->getBall().value();

    robotCommand.angle = (_ball->getPos() - _robot->getPos()).toAngle();
    if (fabs(robotAngleTowardsBallVel) > M_PI * 0.05) {
        Vector2 targetVelIncrease = _ball->getVelocity().stretchToLength(std::max(1.0, fabs((_robot->getPos() - _ball->getPos()).toAngle() - _ball->getVelocity().toAngle()))) / 2;

        LineSegment driveLine = {_robot->getPos(), _robot->getPos() + (robotCommand.vel + targetVelIncrease).stretchToLength(_robot->getVel().length() * 2.0)};
        if (!isCrashingIntoOpponentRobot(driveLine, _robot) && !isCrashingOutsideField(driveLine, _robot)) {
            robotCommand.vel += targetVelIncrease;
        }
    } else if (ballToProjectionDistance / _ball->getVelocity().length() > 0.8) {
        robotCommand.vel -= _ball->getVelocity().stretchToLength(std::max(1.0, _ball->getVelocity().length()));
    }

    return robotCommand;
}

RobotCommand BallHandlePosControl::goToIdleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget, world_new::view::RobotView _robot) {
    Vector2 numTreesTarget;
    auto _ball = world_new::World::instance()->getWorld()->getBall().value();

    Vector2 ballToTarget = targetBallPos - _ball->getPos();
    if (travelStrategy == BACKWARDS) {
        numTreesTarget = _ball->getPos() + ballToTarget.stretchToLength(MAX_BALL_DISTANCE);
    } else if (travelStrategy == FORWARDS) {
        numTreesTarget = _ball->getPos() + ballToTarget.stretchToLength(-MAX_BALL_DISTANCE);
    } else if (ballIsFarFromTarget) {
        numTreesTarget = _ball->getPos() + ballToTarget.stretchToLength(-MAX_BALL_DISTANCE);
    } else {
        numTreesTarget = _ball->getPos() + ballToTarget.stretchToLength(MAX_BALL_DISTANCE);
    }
    auto robotCommand = NumTreePosControl::getRobotCommand(_robot->getId(), numTreesTarget);
    if ((_ball->getPos() - _robot->getPos()).length() < MAX_BALL_DISTANCE * 1.5) {
        robotCommand.angle = (_ball->getPos() - _robot->getPos()).toAngle();
        robotCommand.dribbler = 31;
    }
    return robotCommand;
}

RobotCommand BallHandlePosControl::controlWithPID(PID &xpid, PID &ypid, const RobotCommand &robotCommand, world_new::view::RobotView _robot) {
    RobotCommand pidCommand = robotCommand;
    pidCommand.vel.x = xpid.getOutput(_robot->getVel().x, robotCommand.vel.x);
    pidCommand.vel.y = ypid.getOutput(_robot->getVel().y, robotCommand.vel.y);
    double minVel = 0.112;
    if (pidCommand.vel.length() < minVel) {
        pidCommand.vel = pidCommand.vel.stretchToLength(std::max(minVel, pidCommand.vel.length() + ++ticksNotMoving * 0.006789));
    } else {
        ticksNotMoving = 0;
    }
    return pidCommand;
}

void BallHandlePosControl::updatePID(pidVals newPID) {
    if (newPID != lastPid) {
        lastPid = newPID;
        xBallHandlePID.setPID(newPID, 1.0);
        yBallHandlePID.setPID(newPID, 1.0);
    }
}

bool BallHandlePosControl::isCrashingIntoOpponentRobot(const LineSegment &driveLine, world_new::view::RobotView _robot) {
    double maxCrashVel = 1.0;
    if (_robot->getVel().length() < maxCrashVel || driveLine.length() < maxCrashVel) {
        return false;
    }

    double safeMargin = 0.4;
    auto theirRobots = world_new::World::instance()->getWorld()->getThem();
    for (auto &robot : theirRobots) {
        if (driveLine.distanceToLine(robot->getPos()) > safeMargin) {
            continue;
        }

        if (fabs((driveLine.end - driveLine.start).toAngle() - (robot->getPos() - driveLine.start).toAngle()) > M_PI_2) {
            continue;
        }

        interface::Input::drawData(interface::Visual::BALL_HANDLING, {robot->getPos()}, Qt::red, robot->getId(), interface::Drawing::CIRCLES, 24, 24, 12);
        interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::red, robot->getId(), interface::Drawing::LINES_CONNECTED);
        return true;
    }
    interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::blue, _robot->getId(), interface::Drawing::LINES_CONNECTED);
    return false;
}

bool BallHandlePosControl::isCrashingOutsideField(const LineSegment &driveLine, world_new::view::RobotView _robot) {
    if (!FieldComputations::pointIsInField(*world_new::World::instance()->getField(), driveLine.end)) {
        interface::Input::drawData(interface::Visual::BALL_HANDLING,
                                   {control::ControlUtils::projectPositionToWithinField(*world_new::World::instance()->getField(), driveLine.end)}, Qt::red, _robot->getId(),
                                   interface::Drawing::CIRCLES, 24, 24, 12);
        interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::red, _robot->getId(), interface::Drawing::LINES_CONNECTED);

        return true;
    }

    double maxRobotVel = 3.0;
    if (!FieldComputations::pointIsInField(*world_new::World::instance()->getField(), driveLine.start + (driveLine.end - driveLine.start) * 2)) {
        if (_robot->getVel().length() > maxRobotVel) {
            interface::Input::drawData(interface::Visual::BALL_HANDLING,
                                       {control::ControlUtils::projectPositionToWithinField(*world_new::World::instance()->getField(), driveLine.end)}, Qt::red, _robot->getId(),
                                       interface::Drawing::CIRCLES, 24, 24, 12);
            interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::red, _robot->getId(), interface::Drawing::LINES_CONNECTED);

            return true;
        }
    }

    interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::blue, _robot->getId(), interface::Drawing::LINES_CONNECTED);

    return false;
}

}  // namespace rtt::ai::control