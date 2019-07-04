//
// Created by mrlukasbos on 24-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/control/PositionUtils.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "ShotController.h"
#include "../ballHandling/BallHandlePosControl.h"

namespace rtt {
namespace ai {
namespace control {

/// return a ShotData (which contains data for robotcommands) for a specific robot to shoot at a specific target.
RobotCommand ShotController::getRobotCommand(world::Robot robot, const Vector2 &shotTarget, bool chip,
        BallSpeed ballspeed, bool useAutoGeneva, ShotPrecision precision, int fixedGeneva) {
    // we only allow the external command to change the target if we are not already shooting. Otherwise we use the previous command sent
    if (! isShooting) {
        aimTarget = shotTarget;
        kickerOnTicks = 0;
    }
    auto ball = world::world->getBall();

    // only get a new geneva state if we are allowed to get one
    bool robotAlreadyVeryClose = (robot.pos - ball->pos).length() < 3.0*Constants::ROBOT_RADIUS();
    int currentDesiredGeneva = robot.getGenevaState();

    if (robot.hasWorkingGeneva() && robot.isGenevaReady()) {
        // If using fixed geneva, set it to that
        if (fixedGeneva >= 1 && fixedGeneva <= 5) {
            currentDesiredGeneva = fixedGeneva;
        }
        else if (useAutoGeneva && ! robotAlreadyVeryClose) {
            currentDesiredGeneva = determineOptimalGenevaState(robot, aimTarget);
        }
    }

    if (chip) {
        currentDesiredGeneva = 3;
    }

    Vector2 behindBallPosition = ball->pos + getPlaceBehindBallForGenevaState(robot, aimTarget, currentDesiredGeneva);

    // make a line, on which we can drive straight to it

    std::pair<Vector2, Vector2> lineToDriveOver = std::make_pair(behindBallPosition, ball->pos);
    lineToDriveOver = shiftLineForGeneva(lineToDriveOver, currentDesiredGeneva);
    // check the properties
    bool isOnLineToBall = onLineToBall(robot, lineToDriveOver, precision);
    bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(0.80, shotTarget, robot.pos, 0.3);
    bool validAngle = robotAngleIsGood(robot, lineToDriveOver, precision);

    RobotCommand shotData;
    if (isBehindBall && ((isOnLineToBall && validAngle) || isShooting)) {
        if (! robot.isGenevaReady() && ! chip) {
            isShooting = false;
            // just stand still at the right angle
            shotData.vel = {0.0, 0.0};
            shotData.angle = (lineToDriveOver.second - lineToDriveOver.first).angle();
            std::cout << "Not shooting because geneva is turning" << std::endl;
            kickerOnTicks = 0;
        }
        else {
            isShooting = true;

            shotData = moveStraightToBall(robot, lineToDriveOver);
            bool hasBall = world::world->ourRobotHasBall(robot.id, Constants::MAX_KICK_RANGE());
            if (robot.hasWorkingBallSensor()) {
                shotData = shoot(shotData, robot, lineToDriveOver, aimTarget, chip, ballspeed);
            }
            else if (hasBall) {
                shotData = shoot(shotData, robot, lineToDriveOver, aimTarget, chip, ballspeed);
                shotData.vel = shotData.vel.stretchToLength(0.18); // lower speed to kick to ball correctly
            }
        }
    }
    else {
        kickerOnTicks = 0;
        isShooting = false;
        shotData = goToPlaceBehindBall(robot, lineToDriveOver.first, lineToDriveOver, currentDesiredGeneva);
    }

    interface::Input::drawData(interface::Visual::SHOTLINES, {ball->pos, aimTarget}, Qt::yellow, robot.id,
            interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::DEBUG, {lineToDriveOver.first, lineToDriveOver.second}, Qt::red,
            robot.id, interface::Drawing::LINES_CONNECTED);

    // if we are chipping then the geneva state of 3 was actually just a way of proper positioning
    // we can secretly just keep it's state
    if (chip) currentDesiredGeneva = robot.getGenevaState();


    // Make sure the Geneva state is always correct
    shotData.geneva = currentDesiredGeneva;

    return shotData;
}

/// check if a robot is on a line to a ball
bool ShotController::onLineToBall(const world::Robot &robot, const std::pair<Vector2, Vector2> &line,
        ShotPrecision precision) {
    double dist = ControlUtils::distanceToLine(robot.pos, line.first, line.second);
    if (precision == HIGH) {
        return dist < 0.03;
    }
    else if (precision == MEDIUM) {
        return dist < 0.05;
    }
    return dist < 0.08;
}

/// return the place behind the ball targeted towards the ball target position
Vector2 ShotController::getPlaceBehindBall(const world::Robot &robot, const Vector2 &shotTarget) {
    Vector2 ballPos = world::world->getBall()->pos;
    Vector2 preferredShotVector = ballPos - shotTarget;
    double distanceBehindBall = 2.0*Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    return ballPos + preferredShotVector.stretchToLength(distanceBehindBall);
}

/// use Numtree GTP to go to a place behind the ball
RobotCommand ShotController::goToPlaceBehindBall(const world::Robot &robot, const Vector2 &robotTargetPosition,
        const std::pair<Vector2, Vector2> &line, int geneva) {

    Vector2 genevaAimTarget = updateGenevaAimTarget(geneva);
    auto shotData = robot.getBallHandlePosControl()->getRobotCommand(std::make_shared<world::Robot>(robot),
            genevaAimTarget, robot.angle, control::BallHandlePosControl::TravelStrategy::FORWARDS);

    //TODO: if (rotating to this angle from current angle will hit ball) then pva.angle=angle towards ball
    if ((robot.pos - robotTargetPosition).length() < 0.2) {
        shotData.angle = (line.second - line.first).toAngle();
    }

    if (shotData.vel.length() < 0.15) {
        shotData.vel = shotData.vel.stretchToLength(0.15);
    }

    return shotData;
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
RobotCommand ShotController::moveStraightToBall(world::Robot robot,
        const std::pair<Vector2, Vector2> &lineToDriveOver) {
    /*
     * Moving straight to the ball should be possible by driving forward at a small velocity as soon as you are at the
     * point behind the ball. That is, under normal circumstances. If one if the wheels resists more than the others,
     * the robot might drift and miss the ball. To solve that issue, a PID controller is used to pull the robot towards
     * the line behind the ball.
     */

    RobotCommand robotCommand;

    Vector2 vel = (lineToDriveOver.second - lineToDriveOver.first).stretchToLength(
            0.3); // small constant velocity along the lineToDriveOver
    Vector2 lineUnitVector = (lineToDriveOver.second
            - lineToDriveOver.first).normalize(); // unit vector in the direction of the lineToDriveOver

    Vector2 projection = robot.pos.project(lineToDriveOver.first, lineToDriveOver.second);
    Vector2 err = projection - robot.pos;

    // check on which side of the line the robot is
    double angle = ((robot.pos - lineToDriveOver.first).toAngle() - lineUnitVector.toAngle());
    double sign = angle == 0 ? 1 : angle/abs(angle);

    // use PID to compensate for the error with respect to the line
    //auto newPid = interface::Output::getNumTreePid();
    auto newPidValues = interface::Output::getShotControllerPID();
    updatePid(newPidValues);
    double pidOutput = pid.getOutput(err.length()*sign, 0);

    robotCommand.vel = vel + err.stretchToLength(abs(pidOutput));
//    robotCommand.vel = robotCommand.vel.stretchToLength(robotCommand.vel.length() > 0.3 ? 0.3 : 1);
    robotCommand.angle = (lineToDriveOver.second - lineToDriveOver.first).toAngle();
    return robotCommand;
}

/// Now we should have the ball and kick it.
RobotCommand ShotController::shoot(RobotCommand shotData, const world::Robot &robot,
        const std::pair<Vector2, Vector2> &driveLine, const Vector2 &shotTarget, bool chip,
        BallSpeed desiredBallSpeed) {

    auto ball = world::world->getBall();

    // set the kicker and kickforce
    if (chip) {
        shotData.chipper = true;
        shotData.kicker = false;

        // TODO calibrate chip speed
        shotData.kickerVel = determineKickForce(ball->pos.dist(shotTarget), desiredBallSpeed);
    }
    else {
        shotData.chipper = false;
        shotData.kicker = true;
        shotData.kickerVel = determineKickForce(ball->pos.dist(shotTarget), desiredBallSpeed);
    }

    shotData.kickerForced = kickerOnTicks ++ > 20 && robot.hasBall();
    return shotData;
}

/// Determine how fast we should kick for a pass at a given distance
double ShotController::determineKickForce(double distance, BallSpeed desiredBallSpeed) {
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();

    double velocity = 0;
    switch (desiredBallSpeed) {
    case DRIBBLE_KICK: {
        velocity = sqrt(distance)*rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5);
        break;
    }
    case BALL_PLACEMENT: {
        if (distance > 2.5) {
            velocity = Constants::GRSIM() ? 6.01 : 2.01;
        }
        else {
            velocity = Constants::GRSIM() ? 3.01 : 1.01;
        }
        break;
    }
    case PASS: {
        if (distance >= maxPowerDist) {
            velocity = Constants::MAX_KICK_POWER();
        }
        else if (Constants::GRSIM()) {
            velocity = std::min(1.4*distance/maxPowerDist*Constants::MAX_KICK_POWER(),
                    Constants::DEFAULT_KICK_POWER());
        }
        else {
            velocity = std::min(distance/maxPowerDist*Constants::MAX_KICK_POWER(),
                    Constants::DEFAULT_KICK_POWER()*0.7);
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

bool ShotController::robotAngleIsGood(world::Robot &robot,
        const std::pair<Vector2, Vector2> &lineToDriveOver, ShotPrecision precision) {
    Angle aim((lineToDriveOver.second - lineToDriveOver.first).angle());
    double diff = abs(aim - robot.angle);
    if (precision == HIGH) {
        return diff < toRadians(3);
    }
    if (precision == MEDIUM) {
        return diff < toRadians(6);
    }
    return diff < toRadians(10);
}

// get the place behind the ball as if no geneva is used
// we rotate this vector according to the angle with the shot line
// we intentionally need to remove ball pos because we are rotating the vector
Vector2 ShotController::getPlaceBehindBallForGenevaState(const world::Robot &robot, const Vector2 &shotTarget,
        int genevaState) {
    auto ball = world::world->getBall();
    Vector2 placeStraightBehindBallVector = getPlaceBehindBall(robot, shotTarget) - ball->pos;

    switch (genevaState) {
    case 1:return placeStraightBehindBallVector.rotate(toRadians(20));
    case 2:return placeStraightBehindBallVector.rotate(toRadians(10));
    case 3:return placeStraightBehindBallVector;
    case 4:return placeStraightBehindBallVector.rotate(- toRadians(10));
    case 5:return placeStraightBehindBallVector.rotate(- toRadians(20));
    default:return placeStraightBehindBallVector;
    }
}

std::pair<Vector2, Vector2> ShotController::shiftLineForGeneva(const std::pair<Vector2, Vector2> &line,
        int genevaState) {

    std::pair<Vector2, Vector2> shiftedLine = line;
    if (genevaState < 1 || genevaState > 5) return shiftedLine;

    // move the ball left or right {2,1,0,-1,-2} cm for geneva states {1,2,3,4,5}
    double moveLength = (3 - genevaState)*0.01;

    // get the angle perpendicular to the line, then shift the line by the moveLength
    Angle angle = (line.second - line.first).toAngle() + M_PI_2;
    shiftedLine.first += angle.toVector2(moveLength);
    shiftedLine.second += angle.toVector2(moveLength);
    return shiftedLine;
}

int ShotController::determineOptimalGenevaState(const world::Robot &robot, const Vector2 &shotTarget) {
    auto ball = world::world->getBall();

    // determine the shortest position from where to kick the ball
    Vector2 robotToBall = ball->pos - robot.pos;
    Vector2 preferredShotVector = shotTarget - ball->pos;

    // determine the angle between the robot position and the shot line
    Angle angleWithShotline = robotToBall.toAngle() - preferredShotVector.toAngle();
    if (angleWithShotline.getAngle() > toRadians(20)) {
        return 2;
    }
    else if (angleWithShotline.getAngle() < (- toRadians(20))) {
        return 4;
    }
    return 3;
}

Vector2 ShotController::updateGenevaAimTarget(int geneva) {
    Vector2 ballPos = world::world->getBall()->pos;
    Vector2 ballToAimTarget = aimTarget - ballPos;

    switch (geneva) {
    case 1: return ballPos + ballToAimTarget.rotate(toRadians(20));
    case 2: return ballPos + ballToAimTarget.rotate(toRadians(10));
    case 3: return ballPos + ballToAimTarget;
    case 4: return ballPos + ballToAimTarget.rotate(- toRadians(10));
    case 5: return ballPos + ballToAimTarget.rotate(- toRadians(20));
    default:return ballPos + ballToAimTarget;
    }
}

void ShotController::updatePid(pidVals pidValues) {
    if (lastPid != pidValues) {
        pid.setPID(pidValues);
        lastPid = pidValues;
    }
}

}// control
} // ai
} // rtt
