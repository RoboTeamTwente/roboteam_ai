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

namespace rtt::ai::control {

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea) {
    dribbleForwards = new DribbleForwards(ERROR_MARGIN, ANGLE_ERROR_MARGIN, ballPlacementAccuracy, maxForwardsVelocity);
    dribbleBackwards = new DribbleBackwards(ERROR_MARGIN, ANGLE_ERROR_MARGIN, ballPlacementAccuracy, maxBackwardsVelocity);
    rotateWithBall = new RotateWithBall();
    rotateAroundBall = new RotateAroundBall();

    setCanMoveInDefenseArea(canMoveInDefenseArea);
    setAvoidBallDistance(MAX_BALL_DISTANCE * 0.92);
}

RobotCommand BallHandlePosControl::getRobotCommand(world::World *world, const world::Field *field, const RobotPtr &r, const Vector2 &targetP) {
    this->world = world;
    this->field = field;
    Angle defaultAngle = lockedAngle;
    return BallHandlePosControl::getRobotCommand(world, field, r, targetP, defaultAngle);
}

RobotCommand BallHandlePosControl::getRobotCommand(world::World *world, const world::Field *field, const RobotPtr &r, const Vector2 &targetP, const Angle &targetA,
                                                   TravelStrategy travelStrategy) {
    this->world = world;
    this->field = field;
    TravelStrategy tempTravelStrategy = preferredTravelStrategy;
    preferredTravelStrategy = travelStrategy;
    RobotCommand robotCommand = BallHandlePosControl::getRobotCommand(world, field, r, targetP, targetA);
    preferredTravelStrategy = tempTravelStrategy;

    return robotCommand;
}

/// targetP is the target position of the BALL, targetA is the (final) target angle of the ROBOT
RobotCommand BallHandlePosControl::getRobotCommand(world::World *world, const Field *field, const RobotPtr &r, const Vector2 &targetP, const Angle &targetA) {
    this->world = world;
    this->field = field;

    if (Constants::SHOW_BALL_HANDLE_DEBUG_INFO()) {
        printStatus();
    }

    // update PID values
    pidVals newPidValues = interface::Output::getBallHandlePid();
    updatePID(newPidValues);

    double expectedDelay = 0.04;
    ball = std::make_shared<world::Ball>(world::Ball(*world->getBall()));
    robot = world->getFutureRobot(r, expectedDelay);

    if ((targetPos - targetP).length2() > 0.10) {
        dribbleBackwards->reset();
        dribbleForwards->reset();
    }
    targetPos = targetP;
    targetAngle = targetA;

    // check for ball
    if (!ball) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Can't control the ball with no ball" << std::endl;
        }
        status = FAILURE;
        return {};
    }

    // if the ball is at the targetposition
    bool ballIsAtTargetPosition = (ball->getPos() - targetPos).length2() < ballPlacementAccuracy * ballPlacementAccuracy;
    if (ballIsAtTargetPosition) {
        return finalizeBallHandle();
    }

    lockedAngle = robot->angle;

    // if we do not have the ball yet, go get it
    bool robotDoesNotHaveBall = !robot->hasBall();
    bool robotIsTooFarFromBall = (robot->pos - ball->getPos()).length2() > MAX_BALL_DISTANCE * MAX_BALL_DISTANCE;
    bool robotIsTouchingBall = (robot->pos - ball->getPos()).length() < ROBOT_IS_TOUCHING_BALL;

    bool ballIsMovingTooFast = ball->getVel().length2() > MIN_VEL_FOR_MOVING_BALL * MIN_VEL_FOR_MOVING_BALL;
    bool alreadyDribbling = (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::BackwardsProgress::START ||
                             dribbleForwards->getForwardsProgression() != DribbleForwards::ForwardsProgress::START);

    bool shouldGetBall = alreadyDribbling ? (ballIsMovingTooFast && !robotIsTouchingBall) || (robotDoesNotHaveBall && robotIsTooFarFromBall)
                                          : (ballIsMovingTooFast && !robotIsTouchingBall) || (robotDoesNotHaveBall || robotIsTooFarFromBall);

    bool ballIsOutsideField = !FieldComputations::pointIsInField(*field, ball->getPos());
    if (ballIsOutsideField) {
        status = HANDLING_BALL;
        Vector2 targetBallPos = ControlUtils::projectPositionToWithinField(*field, ball->getPos(), 1.0);
        return handleBall(targetBallPos, BACKWARDS, shouldGetBall);
    }

    bool ballIsFarFromTarget = (targetPos - ball->getPos()).length2() > 0.5;
    bool dribbleBackwardsWhileFarFromTarget =
        (preferredTravelStrategy == NO_PREFERENCE) && (ballIsFarFromTarget) && (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::BackwardsProgress::START);

    if (dribbleBackwardsWhileFarFromTarget) {
        dribbleBackwards->reset();
    }
    return handleBall(targetPos, preferredTravelStrategy, shouldGetBall, ballIsFarFromTarget);
}

RobotCommand BallHandlePosControl::finalizeBallHandle() {
    dribbleBackwards->reset();
    dribbleForwards->reset();

    if (robot->getDribblerState() > 0 || !robot->isDribblerReady()) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Waiting for the dribbler to stop" << std::endl;
        }
        RobotCommand robotCommand;
        robotCommand.vel = {0, 0};
        robotCommand.angle = lockedAngle;
        robotCommand.dribbler = robot->vel.length() < 0.15 ? 0 : 27;
        status = FINALIZING;
        return controlWithPID(xBallHandlePID, yBallHandlePID, robotCommand);
    } else if (fabs(lockedAngle - robot->angle) > ANGLE_ERROR_MARGIN) {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            std::cout << "Rotating robot to final angle" << std::endl;
        }
        status = FINALIZING;
        return controlWithPID(xBallHandlePID, yBallHandlePID, rotateAroundBall->getRobotCommand(robot, targetPos, lockedAngle));
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

RobotCommand BallHandlePosControl::handleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool shouldGoToBall, bool ballIsFarFromTarget) {
    if (shouldGoToBall) {
        status = GET_BALL;
        return goToBall(targetBallPos, travelStrategy, ballIsFarFromTarget);
    }

    status = HANDLING_BALL;
    // check if we are doing something already
    if (dribbleBackwards->getBackwardsProgression() != DribbleBackwards::START) {
        return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleBackwards->getRobotCommand(*field, robot, targetBallPos, targetAngle));
    }

    if (dribbleForwards->getForwardsProgression() != DribbleForwards::START) {
        return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle));
    }

    switch (travelStrategy) {
        case FORWARDS:
            return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle));

        case BACKWARDS:
            return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleBackwards->getRobotCommand(*field, robot, targetBallPos, targetAngle));

        default:
        case NO_PREFERENCE: {
            // choose based on distance from the ball to the target
            if (ballIsFarFromTarget) {
                return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleForwards->getRobotCommand(robot, targetBallPos, targetAngle));
            }
            return controlWithPID(xBallHandlePID, yBallHandlePID, dribbleBackwards->getRobotCommand(*field, robot, targetBallPos, targetAngle));
        }
    }
}

RobotCommand BallHandlePosControl::goToBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget) {
    if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
        std::cout << "we do not have a ball yet" << std::endl;
    }

    dribbleBackwards->reset();
    dribbleForwards->reset();

    bool ballIsMoving = ball->getVel().length2() > MIN_VEL_FOR_MOVING_BALL * MIN_VEL_FOR_MOVING_BALL;

    if (ballIsMoving) {
        return controlWithPID(xGoToBallPID, yGoToBallPID, goToMovingBall());
    }
    return controlWithPID(xGoToBallPID, yGoToBallPID, goToIdleBall(targetBallPos, travelStrategy, ballIsFarFromTarget));
}

RobotCommand BallHandlePosControl::goToMovingBall() {
    Vector2 ballStillPosition = ball->getExpectedBallEndPosition();

    LineSegment ballLine = LineSegment(ball->getPos(), ballStillPosition);
    Vector2 projectionPosition = ballLine.project(robot->pos);
    double robotToProjectionDistance = (projectionPosition - robot->pos).length();
    double ballToProjectionDistance = (projectionPosition - ball->getPos()).length();
    const double AVERAGE_ROBOT_INTERCEPT_VELOCITY = 1.41;  // ms-1

    Angle robotAngleTowardsBallVel = (robot->pos - ball->getPos()).toAngle() - ball->getVel().toAngle();
    bool robotIsBehindBall = fabs(robotAngleTowardsBallVel) < M_PI_4;

    bool robotCanInterceptBall = robotToProjectionDistance / AVERAGE_ROBOT_INTERCEPT_VELOCITY < ballToProjectionDistance / ball->getVel().length();

    RobotCommand robotCommand;
    if (robotIsBehindBall && robotCanInterceptBall) {
        robotCommand = interceptMovingBall(projectionPosition, ballToProjectionDistance, robotAngleTowardsBallVel);
    } else if (!robotIsBehindBall && false) {  // TODO::do not use this till interceptMovingBallTowardsBall() works
        robotCommand = interceptMovingBallTowardsBall();
    } else {
        robotCommand = goBehindBall(ballStillPosition);
    }
    if ((robot->pos - ball->getPos()).length() < 0.5) {
        robotCommand.dribbler = 31;
    }
    return robotCommand;
}

RobotCommand BallHandlePosControl::goBehindBall(const Vector2 &ballStillPosition) {
    Vector2 numTreesTarget = ballStillPosition;
    if (!FieldComputations::pointIsInField(*field, ballStillPosition, -Constants::ROBOT_RADIUS())) {
        LineSegment ballLine = LineSegment(ball->getPos(), ballStillPosition);
        Polygon fieldEdge = FieldComputations::getFieldEdge(*field, -Constants::ROBOT_RADIUS());

        auto intersections = fieldEdge.intersections(ballLine);
        if (intersections.size() == 1) {
            numTreesTarget = intersections[0];
        } else {
            numTreesTarget = ControlUtils::projectPositionToWithinField(*field, ballStillPosition);
        }
    }

    auto robotCommand = NumTreePosControl::getRobotCommand(world, field, robot, numTreesTarget);

    if (getCurrentCollisionWithRobot().getCollisionType() == Collision::BALL) {
        robotCommand.vel = (ball->getPos() - robot->pos).stretchToLength(ball->getVel().length());
    }

    Vector2 targetVelIncrease = ball->getVel();
    LineSegment driveLine = {robot->pos, robot->pos + (robotCommand.vel + targetVelIncrease).stretchToLength(robot->vel.length() * 2.0)};
    if (!isCrashingIntoOpponentRobot(driveLine) && !isCrashingOutsideField(driveLine)) {
        robotCommand.vel += targetVelIncrease;
    }

    return robotCommand;
}

RobotCommand BallHandlePosControl::interceptMovingBallTowardsBall() {
    Angle robotAngleTowardsBall = ball->getVel().toAngle() - (ball->getPos() - robot->pos).toAngle();

    if (fabs(robotAngleTowardsBall) < M_PI * 0.12) {
        LineSegment ntLine = LineSegment(ball->getPos(), ball->getExpectedBallEndPosition());

        if (ntLine.distanceToLine(movingBallTowardsBallTarget) > 0.3) {
            movingBallTowardsBallTarget = ball->getPos() + (ball->getVel()).stretchToLength(std::min(0.5, (ball->getExpectedBallEndPosition() - ball->getPos()).length()));
        }
    } else {
        Line ntLine = Line(ball->getPos(), ball->getExpectedBallEndPosition());
        Vector2 projection = ntLine.project(robot->pos);

        movingBallTowardsBallTarget = ball->getPos() / 2 + projection / 2;
    }

    if (!FieldComputations::pointIsInField(*field, movingBallTowardsBallTarget, -Constants::ROBOT_RADIUS())) {
        LineSegment ballLine = LineSegment(ball->getPos(), movingBallTowardsBallTarget);
        Polygon fieldEdge = FieldComputations::getFieldEdge(*field, -Constants::ROBOT_RADIUS());

        auto intersections = fieldEdge.intersections(ballLine);
        if (intersections.size() == 1) {
            movingBallTowardsBallTarget = intersections[0];
        } else {
            movingBallTowardsBallTarget = ControlUtils::projectPositionToWithinField(*field, ball->getExpectedBallEndPosition());
        }
    }

    auto tempAvoidBallDistance = getAvoidBallDistance();
    setAvoidBallDistance(0.0);

    auto robotCommand = NumTreePosControl::getRobotCommand(world, field, robot, movingBallTowardsBallTarget);

    setAvoidBallDistance(tempAvoidBallDistance);

    if (FieldComputations::pointIsInField(*field, robot->pos + robot->vel)) {
        double ballVel = ball->getVel().length();
        double targetVel = ballVel * 2.4;

        double distanceToBall = (robot->pos - ball->getPos()).length();
        if (distanceToBall < 1.0) {
            targetVel = ballVel * (1.4 + distanceToBall);
        }

        double commandVel = robotCommand.vel.length();
        targetVel = std::max(targetVel, commandVel);

        Vector2 targetVelIncrease = ball->getVel().stretchToLength(targetVel / 2);
        LineSegment driveLine = {robot->pos, robot->pos + targetVelIncrease.stretchToLength(robot->vel.length() * 2.0)};
        if (!isCrashingIntoOpponentRobot(driveLine) && !isCrashingOutsideField(driveLine)) {
            robotCommand.vel += targetVelIncrease;
        }
    }
    if ((robot->pos - ball->getPos()).length() < 0.5) {
        robotCommand.dribbler = 31;
    }
    return robotCommand;
}

RobotCommand BallHandlePosControl::interceptMovingBall(const Vector2 &projectionPosition, double ballToProjectionDistance, const Angle &robotAngleTowardsBallVel) {
    Vector2 numTreesTarget = projectionPosition;
    RobotCommand robotCommand;

    LineSegment driveLine = LineSegment(robot->pos, projectionPosition.stretchToLength(robot->vel.length() * 2.0));
    if (!isCrashingIntoOpponentRobot(driveLine) && !isCrashingOutsideField(driveLine)) {
        robotCommand = BasicPosControl::getRobotCommand(world, field, robot, numTreesTarget);
    } else {
        robotCommand = NumTreePosControl::getRobotCommand(world, field, robot, numTreesTarget);
    }

    robotCommand.angle = (ball->getPos() - robot->pos).toAngle();
    if (fabs(robotAngleTowardsBallVel) > M_PI * 0.05) {
        Vector2 targetVelIncrease = ball->getVel().stretchToLength(std::max(1.0, fabs((robot->pos - ball->getPos()).toAngle() - ball->getVel().toAngle()))) / 2;

        LineSegment driveLine = {robot->pos, robot->pos + (robotCommand.vel + targetVelIncrease).stretchToLength(robot->vel.length() * 2.0)};
        if (!isCrashingIntoOpponentRobot(driveLine) && !isCrashingOutsideField(driveLine)) {
            robotCommand.vel += targetVelIncrease;
        }
    } else if (ballToProjectionDistance / ball->getVel().length() > 0.8) {
        robotCommand.vel -= ball->getVel().stretchToLength(std::max(1.0, ball->getVel().length()));
    }

    return robotCommand;
}

RobotCommand BallHandlePosControl::goToIdleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget) {
    Vector2 numTreesTarget;

    Vector2 ballToTarget = targetBallPos - ball->getPos();
    if (travelStrategy == BACKWARDS) {
        numTreesTarget = ball->getPos() + ballToTarget.stretchToLength(MAX_BALL_DISTANCE);
    } else if (travelStrategy == FORWARDS) {
        numTreesTarget = ball->getPos() + ballToTarget.stretchToLength(-MAX_BALL_DISTANCE);
    } else if (ballIsFarFromTarget) {
        numTreesTarget = ball->getPos() + ballToTarget.stretchToLength(-MAX_BALL_DISTANCE);
    } else {
        numTreesTarget = ball->getPos() + ballToTarget.stretchToLength(MAX_BALL_DISTANCE);
    }
    auto robotCommand = NumTreePosControl::getRobotCommand(world, field, robot, numTreesTarget);
    if ((ball->getPos() - robot->pos).length() < MAX_BALL_DISTANCE * 1.5) {
        robotCommand.angle = (ball->getPos() - robot->pos).toAngle();
        robotCommand.dribbler = 31;
    }
    return robotCommand;
}

RobotCommand BallHandlePosControl::controlWithPID(PID &xpid, PID &ypid, const RobotCommand &robotCommand) {
    RobotCommand pidCommand = robotCommand;
    pidCommand.vel.x = xpid.getOutput(robot->vel.x, robotCommand.vel.x);
    pidCommand.vel.y = ypid.getOutput(robot->vel.y, robotCommand.vel.y);
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

bool BallHandlePosControl::isCrashingIntoOpponentRobot(const LineSegment &driveLine) {
    double maxCrashVel = 1.0;
    if (robot->vel.length() < maxCrashVel || driveLine.length() < maxCrashVel) {
        return false;
    }

    double safeMargin = 0.4;
    auto theirRobots = world->getThem();
    for (auto &robot : theirRobots) {
        if (driveLine.distanceToLine(robot->pos) > safeMargin) {
            continue;
        }

        if (fabs((driveLine.end - driveLine.start).toAngle() - (robot->pos - driveLine.start).toAngle()) > M_PI_2) {
            continue;
        }

        interface::Input::drawData(interface::Visual::BALL_HANDLING, {robot->pos}, Qt::red, robot->id, interface::Drawing::CIRCLES, 24, 24, 12);
        interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::red, robot->id, interface::Drawing::LINES_CONNECTED);
        return true;
    }
    interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::blue, robot->id, interface::Drawing::LINES_CONNECTED);
    return false;
}

bool BallHandlePosControl::isCrashingOutsideField(const LineSegment &driveLine) {
    if (!FieldComputations::pointIsInField(*field, driveLine.end)) {
        interface::Input::drawData(interface::Visual::BALL_HANDLING, {control::ControlUtils::projectPositionToWithinField(*field, driveLine.end)}, Qt::red, robot->id,
                                   interface::Drawing::CIRCLES, 24, 24, 12);
        interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::red, robot->id, interface::Drawing::LINES_CONNECTED);

        return true;
    }

    double maxRobotVel = 3.0;
    if (!FieldComputations::pointIsInField(*field, driveLine.start + (driveLine.end - driveLine.start) * 2)) {
        if (robot->vel.length() > maxRobotVel) {
            interface::Input::drawData(interface::Visual::BALL_HANDLING, {control::ControlUtils::projectPositionToWithinField(*field, driveLine.end)}, Qt::red, robot->id,
                                       interface::Drawing::CIRCLES, 24, 24, 12);
            interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::red, robot->id, interface::Drawing::LINES_CONNECTED);

            return true;
        }
    }

    interface::Input::drawData(interface::Visual::BALL_HANDLING, {driveLine.start, driveLine.end}, Qt::blue, robot->id, interface::Drawing::LINES_CONNECTED);

    return false;
}

}  // namespace rtt::ai::control