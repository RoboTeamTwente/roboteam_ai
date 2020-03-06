//
// Created by mrlukasbos on 24-4-19.
//

#include "control/shot-controllers/ShotController.h"
#include <control/ControlUtils.h>
#include <control/PositionUtils.h>
#include <interface/api/Input.h>
#include <world_new/World.hpp>
#include "control/ball-handling/BallHandlePosControl.h"

namespace rtt::ai::control {

RobotCommand ShotController::getRobotCommand(int robotId, const Vector2 &shotTarget, bool chip, BallSpeed ballspeed, bool useAutoGeneva, ShotPrecision precision, int genevaState) {
    // we only allow the external command to change the target if we are not already shooting. Otherwise we use the previous command sent
    if (!isShooting) {
        aimTarget = shotTarget;
        kickerOnTicks = 0;
    }
    auto ball = world_new::World::instance()->getWorld()->getBall();
    auto robot = world_new::World::instance()->getWorld()->getRobotForId(robotId, true);

    Vector2 behindBallPosition = getPlaceBehindBall(aimTarget, ball->get()->getPos());

    // make a line, on which we can drive straight to it
    std::pair<Vector2, Vector2> lineToDriveOver = std::make_pair(behindBallPosition, ball->get()->getPos());

    // check the properties
    bool isOnLineToBall = onLineToBall(robot.value(), lineToDriveOver, precision);
    bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(0.80, shotTarget, robot->get()->getPos(), 0.3);
    bool validAngle = robotAngleIsGood(robot.value(), lineToDriveOver, precision, ball.value());

    RobotCommand shotData;
    if (isBehindBall && ((isOnLineToBall && validAngle) || isShooting)) {
        isShooting = true;

        shotData = moveStraightToBall(robot.value(), lineToDriveOver);
        bool hasBall = robot->hasBall(Constants::MAX_KICK_RANGE());
        shotData = shoot(shotData, robot.value(), lineToDriveOver, aimTarget, chip, ballspeed);
        shotData.vel = shotData.vel.stretchToLength(std::min(0.18, shotData.vel.length()));  // lower speed to kick to ball correctly
    } else {
        kickerOnTicks = 0;
        isShooting = false;
        shotData = goToPlaceBehindBall(robot.value(), lineToDriveOver.first, lineToDriveOver);
    }

    interface::Input::drawData(interface::Visual::SHOTLINES, {ball.value()->getPos(), aimTarget}, Qt::yellow, robot->get()->getId(), interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::DEBUG, {lineToDriveOver.first, lineToDriveOver.second}, Qt::red, robot->get()->getId(), interface::Drawing::LINES_CONNECTED);

    return shotData;
}

/// check if a robot is on a line to a ball
bool ShotController::onLineToBall(const world_new::view::RobotView robot, const std::pair<Vector2, Vector2> &line, ShotPrecision precision) {
    double dist = ControlUtils::distanceToLine(robot->getPos(), line.first, line.second);
    if (precision == HIGH) {
        return dist < 0.03;
    } else if (precision == MEDIUM) {
        return dist < 0.05;
    }
    return dist < 0.08;
}

/// return the place behind the ball targeted towards the ball target position
Vector2 ShotController::getPlaceBehindBall(const Vector2 &shotTarget, const Vector2 &ballPos) {
    Vector2 preferredShotVector = ballPos - shotTarget;
    double distanceBehindBall = 2.0 * Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    return ballPos + preferredShotVector.stretchToLength(distanceBehindBall);
}

/// use Numtree GTP to go to a place behind the ball
RobotCommand ShotController::goToPlaceBehindBall(const world_new::view::RobotView robot, const Vector2 &robotTargetPosition, const std::pair<Vector2, Vector2> &line) {
    auto shotData = robot->getBallHandlePosControl()->getRobotCommand(robot->getId(), aimTarget, robot->getAngle(), control::BallHandlePosControl::TravelStrategy::FORWARDS);

    // TODO: if (rotating to this angle from current angle will hit ball) then pva.angle=angle towards ball
    if ((robot->getPos() - robotTargetPosition).length() < 0.2) {
        shotData.angle = (line.second - line.first).toAngle();
    }

    if (shotData.vel.length() < 0.15) {
        shotData.vel = shotData.vel.stretchToLength(0.15);
    }

    return shotData;
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
RobotCommand ShotController::moveStraightToBall(world_new::view::RobotView robot, const std::pair<Vector2, Vector2> &lineToDriveOver) {
    /*
     * Moving straight to the ball should be possible by driving forward at a small velocity as soon as you are at the
     * point behind the ball. That is, under normal circumstances. If one if the wheels resists more than the others,
     * the robot might drift and miss the ball. To solve that issue, a PID controller is used to pull the robot towards
     * the line behind the ball.
     */

    RobotCommand robotCommand;

    Vector2 vel = (lineToDriveOver.second - lineToDriveOver.first).stretchToLength(0.3);    // small constant velocity along the lineToDriveOver
    Vector2 lineUnitVector = (lineToDriveOver.second - lineToDriveOver.first).normalize();  // unit vector in the direction of the lineToDriveOver

    Vector2 projection = robot->getPos().project(lineToDriveOver.first, lineToDriveOver.second);
    Vector2 err = projection - robot->getPos();

    // check on which side of the line the robot is
    double angle = ((robot->getPos() - lineToDriveOver.first).toAngle() - lineUnitVector.toAngle());
    double sign = angle == 0 ? 1 : angle / abs(angle);

    // use PID to compensate for the error with respect to the line
    // auto newPid = interface::Output::getNumTreePid();
    auto newPidValues = interface::Output::getShotControllerPID();
    updatePid(newPidValues);
    double pidOutput = pid.getOutput(err.length() * sign, 0);

    robotCommand.vel = vel + err.stretchToLength(abs(pidOutput));
    // robotCommand.vel = robotCommand.vel.stretchToLength(robotCommand.vel.length() > 0.3 ? 0.3 : 1);
    robotCommand.angle = (lineToDriveOver.second - lineToDriveOver.first).toAngle();
    return robotCommand;
}

/// Now we should have the ball and kick it.
RobotCommand ShotController::shoot(RobotCommand shotData, const world_new::view::RobotView robot, const std::pair<Vector2, Vector2> &driveLine, const Vector2 &shotTarget,
                                   bool chip, BallSpeed desiredBallSpeed) {
    auto ball = world_new::World::instance()->getWorld()->getBall().value();

    // set the kicker and kickforce
    if (chip) {
        shotData.chipper = true;
        shotData.kicker = false;

        // TODO calibrate chip speed
        shotData.kickerVel = determineKickForce(ball->getPos().dist(shotTarget), desiredBallSpeed);
    } else {
        shotData.chipper = false;
        shotData.kicker = true;
        shotData.kickerVel = determineKickForce(ball->getPos().dist(shotTarget), desiredBallSpeed);
    }

    if (kickerOnTicks++ > 80) {
        kickerOnTicks = 0;
        shotData.kickerForced = robot.hasBall();
    } else if (!robot->isWorkingBallSensor()) {
        shotData.kickerForced = robot.hasBall(Constants::MAX_KICK_RANGE());
    }
    return shotData;
}

/// Determine how fast we should kick for a pass at a given distance
double ShotController::determineKickForce(double distance, BallSpeed desiredBallSpeed) {
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();

    double velocity = 0;
    switch (desiredBallSpeed) {
        case DRIBBLE_KICK: {
            velocity = sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER() / (sqrt(maxPowerDist) * 1.5);
            break;
        }
        case BALL_PLACEMENT: {
            if (distance > 2.5) {
                velocity = Constants::GRSIM() ? 6.01 : 2.01;
            } else {
                velocity = Constants::GRSIM() ? 3.01 : 1.01;
            }
            break;
        }
        case PASS: {
            if (distance >= maxPowerDist) {
                velocity = Constants::MAX_KICK_POWER();
            } else if (Constants::GRSIM()) {
                velocity = std::min(1.4 * distance / maxPowerDist * Constants::MAX_KICK_POWER(), Constants::DEFAULT_KICK_POWER());
            } else {
                velocity = std::min(distance / maxPowerDist * Constants::MAX_KICK_POWER(), Constants::DEFAULT_KICK_POWER() * 0.7);
            }
            break;
        }
        case MAX_SPEED: {
            velocity = rtt::ai::Constants::MAX_KICK_POWER();
            break;
        }
        default: {
            velocity = rtt::ai::Constants::MAX_KICK_POWER();
        }
    }

    // limit the output to the max kick speed
    return std::min(std::max(velocity, 1.01), rtt::ai::Constants::MAX_KICK_POWER());
}

bool ShotController::robotAngleIsGood(world_new::view::RobotView robot, const std::pair<Vector2, Vector2> &lineToDriveOver, ShotPrecision precision,
                                      world_new::view::BallView ball) {
    Line ballToTarget(ball->getPos(), aimTarget);
    Line robotToBall(robot->getPos(), ball->getPos());
    double diff = {ballToTarget.direction().angle() - robotToBall.direction().angle()};

    if (precision == HIGH) {
        return diff < toRadians(3);
    }
    if (precision == MEDIUM) {
        return diff < toRadians(6);
    }
    return diff < toRadians(10);
}

void ShotController::updatePid(pidVals pidValues) {
    if (lastPid != pidValues) {
        pid.setPID(pidValues);
        lastPid = pidValues;
    }
}

}  // namespace rtt::ai::control
