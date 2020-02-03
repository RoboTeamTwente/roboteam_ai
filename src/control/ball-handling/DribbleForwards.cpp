//
// Created by thijs on 25-5-19.
//

#include "control/ball-handling/DribbleForwards.h"

#include <control/ControlUtils.h>
#include <world/Robot.h>
#include <world/World.h>

#include <sstream>

#include "control/ball-handling/RotateAroundBall.h"
#include "control/ball-handling/RotateWithBall.h"
#include "interface/api/Input.h"
#include "world/Ball.h"

namespace rtt::ai::control {

    RobotCommand DribbleForwards::getRobotCommand(RobotPtr r, const Vector2 &targetP, const Angle &targetA) {
        robot = std::move(r);
        ball = world::world->getBall();
        finalTargetAngle = targetA;
        targetAngle = targetA;
        finalTargetPos = targetP;
        targetPos = targetP;
        updateForwardsProgress();
        return sendForwardsCommand();
    }

    void DribbleForwards::reset() { forwardsProgress = START; }

    void DribbleForwards::updateForwardsProgress() {
        if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
            printForwardsProgress();
        }

        // check if we still have ball
        targetAngle = (finalTargetPos - ball->getPos()).toAngle();
        Angle angleDifference = robot->angle - targetAngle;

        if (forwardsProgress != ForwardsProgress::DRIBBLE_FORWARD) {
            waitingTicks = 0;
        }

        // update forwards progress
        switch (forwardsProgress) {
            case TURNING: {
                targetAngle = (finalTargetPos - ball->getPos()).toAngle();
                if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
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
                if (robot->hasBall()) {
                    forwardsDribbleLine = {robot->pos, finalTargetPos};
                    forwardsProgress = DRIBBLE_FORWARD;
                    return;
                }
                return;
            }
            case DRIBBLE_FORWARD: {
                if (!robot->hasBall()) {
                    forwardsProgress = APPROACHING;
                    return;
                }
                if ((ball->getPos() - finalTargetPos).length2() < ballPlacementAccuracy * ballPlacementAccuracy) {
                    forwardsProgress = SUCCESS;
                    return;
                }
                return;
            }
            case FAIL:return;
            case START:return;
            default:return;
            case SUCCESS: {
                if ((ball->getPos() - finalTargetPos).length2() < ballPlacementAccuracy * ballPlacementAccuracy) {
                    return;
                }
                forwardsProgress = START;
            }
        }
    }

    RobotCommand DribbleForwards::startTravelForwards() {
        lockedAngle = Angle();
        forwardsDribbleLine = {};
        forwardsProgress = TURNING;
        return sendTurnCommand();
    }

    RobotCommand DribbleForwards::sendForwardsCommand() {
        switch (forwardsProgress) {
            case START:return startTravelForwards();
            case TURNING:return sendTurnCommand();
            case APPROACHING:return sendApproachCommand();
            case DRIBBLE_FORWARD:return sendDribbleForwardsCommand();
            case SUCCESS:return sendSuccessCommand();
            case FAIL: {
                forwardsProgress = START;
                return {};
            }
        }
        return {};
    }

    RobotCommand DribbleForwards::sendTurnCommand() {
        if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
            lockedAngle = targetAngle;
        }
        targetPos = finalTargetPos;
        return rotateAroundBall->getRobotCommand(robot, targetPos, targetAngle);
    }

    RobotCommand DribbleForwards::sendApproachCommand() {
        RobotCommand command;
        command.dribbler = 31;
        command.vel = (robot->pos - ball->getPos()).stretchToLength(maxVel);
        command.angle = lockedAngle;
        return command;
    }

    RobotCommand DribbleForwards::sendDribbleForwardsCommand() {
        RobotCommand command;
        command.dribbler = 31;
        command.angle = lockedAngle;
        command.vel = lockedAngle.toVector2(maxVel);
        int ramp = 100;
        if (waitingTicks < ramp) {
            command.vel = command.vel.stretchToLength(maxVel / 4 + 3 * command.vel.length() * waitingTicks++ / (ramp * 4));
        }

        // check if the robot is still on the virtual line from ball->pos to the target
        double maxDist = errorMargin * 8 + std::fmin(1.0, 2 * (robot->pos - finalTargetPos).length());

        if (control::ControlUtils::distanceToLine(robot->pos, forwardsDribbleLine.first, forwardsDribbleLine.second) > maxDist) {
            forwardsProgress = TURNING;
        }

        Angle robotAngleTowardsLine = (finalTargetPos - robot->pos).toAngle() - (forwardsDribbleLine.second - forwardsDribbleLine.first).toAngle();

        double compensationFactor = std::fmax(4.0, 8.0 * (ramp - waitingTicks) / ramp);
        Vector2 compensation = (robot->angle + M_PI_2).toVector2(std::min(robotAngleTowardsLine * compensationFactor, 0.05));
        command.vel += compensation;

        interface::Input::drawData(interface::Visual::BALL_HANDLING, {compensation + robot->pos, robot->pos}, Qt::white, robot->id, interface::Drawing::ARROWS);
        interface::Input::drawData(interface::Visual::BALL_HANDLING, {forwardsDribbleLine.first, forwardsDribbleLine.second}, Qt::white, robot->id,
                                   interface::Drawing::LINES_CONNECTED);

        // limit velocity close to the target
        double distanceToTarget = (finalTargetPos - robot->pos).length();
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
            case START:ss << "start";
                break;
            case TURNING:ss << "turning";
                break;
            case APPROACHING:ss << "approaching";
                break;
            case DRIBBLE_FORWARD:ss << "dribble forwards";
                break;
            case SUCCESS:ss << "success";
                break;
            case FAIL:ss << "fail";
                break;
        }
        std::cout << ss.str() << std::endl;
    }

    DribbleForwards::ForwardsProgress DribbleForwards::getForwardsProgression() { return forwardsProgress; }

    DribbleForwards::DribbleForwards(double errorMargin, double angularErrorMargin, double ballPlacementAccuracy, double maxVel)
        : waitingTicks(0), errorMargin(errorMargin), angleErrorMargin(angularErrorMargin), ballPlacementAccuracy(ballPlacementAccuracy), maxVel(maxVel) {
        robot = std::make_shared<world::Robot>(world::Robot());
        ball = std::make_shared<world::Ball>(world::Ball());

        rotateAroundBall = new RotateAroundBall();
        rotateAroundRobot = new RotateWithBall();
    }

    DribbleForwards::~DribbleForwards() {
        delete rotateAroundBall;
        delete rotateAroundRobot;
    }

    void DribbleForwards::setMaxVel(double maxV) { maxVel = maxV; }

}  // namespace rtt::ai::control