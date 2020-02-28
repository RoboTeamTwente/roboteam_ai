//
// Created by mrlukasbos on 24-4-19.
//

#include "control/shot-controllers/ShotController.h"

#include <control/ControlUtils.h>
#include <control/PositionUtils.h>
#include <interface/api/Input.h>

namespace rtt::ai::control {

/// return a ShotData (which contains data for robotcommands) for a specific robot to shoot at a specific target.
    RobotCommand
    ShotController::getRobotCommand(const Field &field, world_new::view::RobotView &robot, const Vector2 &shotTarget,
                                    bool chip, BallSpeed ballspeed, ShotPrecision precision,
                                    world_new::view::BallView &ball, world_new::view::WorldDataView &world) {
    // we only allow the external command to change the target if we are not already shooting. Otherwise we use the previous command sent
    if (!isShooting) {
        aimTarget = shotTarget;
        kickerOnTicks = 0;
    }
    //
    //    // only get a new geneva state if we are allowed to get one
    //    bool robotAlreadyVeryClose = (robot->getPos() - ball->getPos()).length() < 3.0 * Constants::ROBOT_RADIUS();
    //

    Vector2 behindBallPosition = ball->getPos() + getPlaceBehindBall(robot, aimTarget);

    // make a line, on which we can drive straight to it

    std::pair<Vector2, Vector2> lineToDriveOver = std::make_pair(behindBallPosition, ball->getPos());
    // check the properties
    bool isOnLineToBall = onLineToBall(robot, lineToDriveOver, precision);
    bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(ball, 0.80, shotTarget, robot->getPos(), 0.3);
    bool validAngle = robotAngleIsGood(robot, lineToDriveOver, precision);

    RobotCommand shotData;
    if (isBehindBall && ((isOnLineToBall && validAngle) || isShooting)) {
        if (!chip) {
            isShooting = false;
            // just stand still at the right angle
            shotData.vel = {0.0, 0.0};
            shotData.angle = (lineToDriveOver.second - lineToDriveOver.first).angle();
            std::cout << "Not shooting because geneva is turning" << std::endl;
            kickerOnTicks = 0;
        } else {
            isShooting = true;

            shotData = moveStraightToBall(robot, lineToDriveOver);
            bool hasBall = robot.hasBall();
            if (robot->isWorkingBallSensor() || hasBall) {
                shotData = shoot(shotData, robot, lineToDriveOver, aimTarget, chip, ballspeed);
                shotData.vel = shotData.vel.stretchToLength(std::min(0.18, shotData.vel.length()));  // lower speed to kick to ball correctly
            }
        }
    } else {
        kickerOnTicks = 0;
        isShooting = false;
        shotData = goToPlaceBehindBall(field, robot, lineToDriveOver.first, lineToDriveOver, world);
    }

    interface::Input::drawData(interface::Visual::SHOTLINES, {ball->getPos(), aimTarget}, Qt::yellow, robot->getId(), interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::DEBUG, {lineToDriveOver.first, lineToDriveOver.second}, Qt::red, robot->getId(), interface::Drawing::LINES_CONNECTED);

    return shotData;
}

/// check if a robot is on a line to a ball
bool ShotController::onLineToBall(const world_new::view::RobotView &robot, const std::pair<Vector2, Vector2> &line, ShotPrecision precision) {
    double dist = ControlUtils::distanceToLine(robot->getPos(), line.first, line.second);
    if (precision == HIGH) {
        return dist < 0.03;
    } else if (precision == MEDIUM) {
        return dist < 0.05;
    }
    return dist < 0.08;
}

/// return the place behind the ball targeted towards the ball target position
Vector2 ShotController::getPlaceBehindBall(const world_new::view::RobotView &robot, const Vector2 &shotTarget) {
    Vector2 ballPos = world::world->getBall()->getPos();
    Vector2 preferredShotVector = ballPos - shotTarget;
    double distanceBehindBall = 2.0 * Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    return ballPos + preferredShotVector.stretchToLength(distanceBehindBall);
}



/// At this point we should be behind the ball. now we can move towards the ball to kick it.
RobotCommand ShotController::moveStraightToBall(world_new::view::RobotView &robot, const std::pair<Vector2, Vector2> &lineToDriveOver) {
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
RobotCommand ShotController::shoot(RobotCommand shotData, const world_new::view::RobotView &robot, const std::pair<Vector2, Vector2> &driveLine, const Vector2 &shotTarget,
                                   bool chip, BallSpeed desiredBallSpeed) {
    auto ball = world::world->getBall();

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
        default: { velocity = rtt::ai::Constants::MAX_KICK_POWER(); }
    }

    // limit the output to the max kick speed
    return std::min(std::max(velocity, 1.01), rtt::ai::Constants::MAX_KICK_POWER());
}

bool ShotController::robotAngleIsGood(world_new::view::RobotView &robot, const std::pair<Vector2, Vector2> &lineToDriveOver, ShotPrecision precision) {
    Angle aim((lineToDriveOver.second - lineToDriveOver.first).angle());
    double diff = abs(aim - robot->getAngle());
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

RobotCommand ShotController::goToPlaceBehindBall(const Field &field, const world_new::view::RobotView &robot,
                                                 const Vector2 &robotTargetPosition,
                                                 const std::pair<Vector2, Vector2> &driveLine,
                                                 world_new::view::WorldDataView &world) {

    auto shotData = robot.getControllers().getBallHandlePosController()->getRobotCommand(&world, &field, robot, robotTargetPosition, robot->getAngle(), BallHandlePosControl::TravelStrategy::FORWARDS);
    // TODO: if (rotating to this angle from current angle will hit ball) then pva.angle=angle towards ball
    if ((robot->getPos() - robotTargetPosition).length() < 0.2) {
        shotData.angle = (driveLine.second - driveLine.first).toAngle();
    }

    if (shotData.vel.length() < 0.15) {
        shotData.vel = shotData.vel.stretchToLength(0.15);
    }

    return shotData;
}

}  // namespace rtt::ai::control
