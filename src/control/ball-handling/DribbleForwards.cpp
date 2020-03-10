//
// Created by thijs on 25-5-19.
//

#include "control/ball-handling/DribbleForwards.h"
#include <control/ControlUtils.h>
#include <sstream>
#include "control/ball-handling/RotateAroundBall.h"
#include "control/ball-handling/RotateWithBall.h"
#include "interface/api/Input.h"
#include "world_new/World.hpp"

namespace rtt::ai::control {

RobotCommand DribbleForwards::getRobotCommand(world_new::view::RobotView _robot, const Vector2 &targetP, const Angle &targetA) {
    finalTargetAngle = targetA;
    targetAngle = targetA;
    finalTargetPos = targetP;
    targetPos = targetP;
    updateForwardsProgress(_robot);
    return sendForwardsCommand(_robot);
}

void DribbleForwards::reset() { forwardsProgress = START; }

void DribbleForwards::updateForwardsProgress(world_new::view::RobotView _robot) {
    auto _ball = world_new::World::instance()->getWorld()->getBall().value();
    if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
        printForwardsProgress();
    }

    // check if we still have ball
    targetAngle = (finalTargetPos - _ball->getPos()).toAngle();
    Angle angleDifference = _robot->getAngle() - targetAngle;

    if (forwardsProgress != ForwardsProgress::DRIBBLE_FORWARD) {
        waitingTicks = 0;
    }

    // update forwards progress
    switch (forwardsProgress) {
        case TURNING: {
            targetAngle = (finalTargetPos - _ball->getPos()).toAngle();
            if (fabs(targetAngle - _robot->getAngle()) < angleErrorMargin) {
                lockedAngle = targetAngle;
                forwardsProgress = APPROACHING;
            }
            return;
        }
        case APPROACHING: {
            if (fabs(angleDifference) > angleErrorMargin) {
                forwardsProgress = TURNING;
                return;
            }
            if (_robot.hasBall()) {
                forwardsDribbleLine = {_robot->getPos(), finalTargetPos};
                forwardsProgress = DRIBBLE_FORWARD;
                return;
            }
            return;
        }
        case DRIBBLE_FORWARD: {
            if (!_robot.hasBall()) {
                forwardsProgress = APPROACHING;
                return;
            }
            if ((_ball->getPos() - finalTargetPos).length2() < ballPlacementAccuracy * ballPlacementAccuracy) {
                forwardsProgress = SUCCESS;
                return;
            }
            return;
        }
        case FAIL:
            return;
        case START:
            return;
        default:
            return;
        case SUCCESS: {
            if ((_ball->getPos() - finalTargetPos).length2() < ballPlacementAccuracy * ballPlacementAccuracy) {
                return;
            }
            forwardsProgress = START;
        }
    }
}

RobotCommand DribbleForwards::startTravelForwards(world_new::view::RobotView _robot) {
    lockedAngle = Angle();
    forwardsDribbleLine = {};
    forwardsProgress = TURNING;
    return sendTurnCommand(_robot);
}

RobotCommand DribbleForwards::sendForwardsCommand(world_new::view::RobotView _robot) {
    switch (forwardsProgress) {
        case START:
            return startTravelForwards(_robot);
        case TURNING:
            return sendTurnCommand(_robot);
        case APPROACHING:
            return sendApproachCommand(_robot);
        case DRIBBLE_FORWARD:
            return sendDribbleForwardsCommand(_robot);
        case SUCCESS:
            return sendSuccessCommand();
        case FAIL: {
            forwardsProgress = START;
            return {};
        }
    }
    return {};
}

RobotCommand DribbleForwards::sendTurnCommand(world_new::view::RobotView _robot) {
    if (fabs(targetAngle - _robot->getAngle()) < angleErrorMargin) {
        lockedAngle = targetAngle;
    }
    targetPos = finalTargetPos;
    return rotateAroundBall->getRobotCommand(_robot, targetPos, targetAngle);
}

RobotCommand DribbleForwards::sendApproachCommand(world_new::view::RobotView _robot) {
    RobotCommand command;
    auto _ball = world_new::World::instance()->getWorld()->getBall().value();
    command.dribbler = 31;
    command.vel = (_robot->getPos() - _ball->getPos()).stretchToLength(maxVel);
    command.angle = lockedAngle;
    return command;
}

RobotCommand DribbleForwards::sendDribbleForwardsCommand(world_new::view::RobotView _robot) {
    RobotCommand command;
    command.dribbler = 31;
    command.angle = lockedAngle;
    command.vel = lockedAngle.toVector2(maxVel);
    int ramp = 100;
    if (waitingTicks < ramp) {
        command.vel = command.vel.stretchToLength(maxVel / 4 + 3 * command.vel.length() * waitingTicks++ / (ramp * 4));
    }

    // check if the robot is still on the virtual line from ball->pos to the target
    double maxDist = errorMargin * 8 + std::fmin(1.0, 2 * (_robot->getPos() - finalTargetPos).length());

    if (control::ControlUtils::distanceToLine(_robot->getPos(), forwardsDribbleLine.first, forwardsDribbleLine.second) > maxDist) {
        forwardsProgress = TURNING;
    }

    Angle robotAngleTowardsLine = (finalTargetPos - _robot->getPos()).toAngle() - (forwardsDribbleLine.second - forwardsDribbleLine.first).toAngle();

    double compensationFactor = std::fmax(4.0, 8.0 * (ramp - waitingTicks) / ramp);
    Vector2 compensation = (_robot->getAngle() + M_PI_2).toVector2(std::min(robotAngleTowardsLine * compensationFactor, 0.05));
    command.vel += compensation;

    interface::Input::drawData(interface::Visual::BALL_HANDLING, {compensation + _robot->getPos(), _robot->getPos()}, Qt::white, _robot->getId(), interface::Drawing::ARROWS);
    interface::Input::drawData(interface::Visual::BALL_HANDLING, {forwardsDribbleLine.first, forwardsDribbleLine.second}, Qt::white, _robot->getId(),
                               interface::Drawing::LINES_CONNECTED);

    // limit velocity close to the target
    double distanceToTarget = (finalTargetPos - _robot->getPos()).length();
    if (distanceToTarget < 1.0) {
        command.vel = command.vel.stretchToLength(std::max(0.2, distanceToTarget * maxVel));
    }
    return command;
}

RobotCommand DribbleForwards::sendSuccessCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.angle = lockedAngle;
    return command;
}

void DribbleForwards::printForwardsProgress() {
    std::stringstream ss;
    ss << "forwards progress:                  ";
    switch (forwardsProgress) {
        case START:
            ss << "start";
            break;
        case TURNING:
            ss << "turning";
            break;
        case APPROACHING:
            ss << "approaching";
            break;
        case DRIBBLE_FORWARD:
            ss << "dribble forwards";
            break;
        case SUCCESS:
            ss << "success";
            break;
        case FAIL:
            ss << "fail";
            break;
    }
    std::cout << ss.str() << std::endl;
}

DribbleForwards::ForwardsProgress DribbleForwards::getForwardsProgression() { return forwardsProgress; }

DribbleForwards::DribbleForwards(double errorMargin, double angularErrorMargin, double ballPlacementAccuracy, double maxVel)
    : waitingTicks(0), errorMargin(errorMargin), angleErrorMargin(angularErrorMargin), ballPlacementAccuracy(ballPlacementAccuracy), maxVel(maxVel) {

    rotateAroundBall = new RotateAroundBall();
    rotateAroundRobot = new RotateWithBall();
}

DribbleForwards::~DribbleForwards() {
    delete rotateAroundBall;
    delete rotateAroundRobot;
}

void DribbleForwards::setMaxVel(double maxV) { maxVel = maxV; }

}  // namespace rtt::ai::control