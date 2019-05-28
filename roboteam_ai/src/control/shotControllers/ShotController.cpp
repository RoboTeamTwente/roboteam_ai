//
// Created by mrlukasbos on 24-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/control/PositionUtils.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "ShotController.h"

namespace rtt {
namespace ai {
namespace control {

/// return a ShotData (which contains data for robotcommands) for a specific robot to shoot at a specific target.
ShotData ShotController::getShotData(world::Robot robot, Vector2 shotTarget, bool chip, BallSpeed ballspeed,
        bool useAutoGeneva, ShotPrecision precision) {
    // we only allow the external command to change the target if we are not already shooting. Otherwise we use the previous command sent
    if (! isShooting) {
        aimTarget = shotTarget;
    }
    auto ball = world::world->getBall();

    // only get a new geneva state if we are allowed to get one
    bool robotAlreadyVeryClose = robot.pos.dist(ball->pos) < 3.0*Constants::ROBOT_RADIUS();
    int currentDesiredGeneva = robot.getGenevaState();

    if (useAutoGeneva && robot.hasWorkingGeneva() && ! genevaIsTurning && ! robotAlreadyVeryClose) {
        currentDesiredGeneva = determineOptimalGenevaState(robot, aimTarget);
    }

    if (chip) {
        currentDesiredGeneva = 3;
    }

    Vector2 futureBallPos = ball->pos;
    Vector2 behindBallPosition =
            futureBallPos + getPlaceBehindBallForGenevaState(robot, aimTarget, currentDesiredGeneva);

    // make a line, on which we can drive straight to it

    std::pair<Vector2, Vector2> lineToDriveOver = std::make_pair(behindBallPosition, ball->pos);
    lineToDriveOver = shiftLineForGeneva(lineToDriveOver, currentDesiredGeneva);
    // check the properties
    bool isOnLineToBall = onLineToBall(robot, lineToDriveOver, precision);
    bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(0.80, shotTarget, robot.pos, 0.3);
    bool validAngle = robotAngleIsGood(robot, lineToDriveOver, precision);

    ShotData shotData;
   // std::cout<<" Online: "<<isOnLineToBall <<" behind: "<<isBehindBall<<" valid: "<<validAngle<<" isShooting: " <<isShooting<<std::endl;

    if (isOnLineToBall && isBehindBall && (validAngle || isShooting)) {
        if (genevaIsTurning) {
            isShooting = false;
            // just stand still at the right angle
            shotData.vel = {0.0, 0.0};
            shotData.angle = (lineToDriveOver.second - lineToDriveOver.first).angle();
            std::cout << "Not shooting because geneva is turning for " << secondsToTurnGeneva << "s" << std::endl;
        }
        else {
            isShooting = true;
            shotData=moveAndShootGrSim(robot,chip,lineToDriveOver,ballspeed);

            //shotData = Constants::GRSIM() ? moveAndShootGrSim(robot, chip, lineToDriveOver, ballspeed)
              //      : moveAndShoot(robot, chip, lineToDriveOver, ballspeed);
        }
    }
    else {
        isShooting = false;
        shotData = goToPlaceBehindBall(robot, lineToDriveOver.first + ball->vel*0.2, lineToDriveOver);
    }

    interface::Input::drawData(interface::Visual::SHOTLINES, {ball->pos, aimTarget}, Qt::yellow, robot.id,
            interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::DEBUG, {lineToDriveOver.first, lineToDriveOver.second}, Qt::red,
            robot.id, interface::Drawing::LINES_CONNECTED);
    // Make sure the Geneva state is always correct
    shotData.genevaState = currentDesiredGeneva;
    return shotData;
}

void ShotController::setGenevaDelay(int genevaDifference) {
    if (genevaDifference != 0) {
        genevaIsTurning = true;
        // each turn should increase the time which the geneva is turning
        secondsToTurnGeneva = genevaDifference*0.4;
        lastTimeGenevaChanged = ros::Time::now().toSec();
    }
}

/// check if a robot is on a line to a ball

bool ShotController::onLineToBall(const world::Robot &robot, std::pair<Vector2, Vector2> line,
        ShotPrecision precision) {
    double dist = ControlUtils::distanceToLine(robot.pos, line.first, line.second);
    if (precision == HIGH) {
        return dist < 0.04;
    }
    else if (precision == MEDIUM) {
        return dist < 0.05;
    }
    return dist < 0.08;
}

/// return the place behind the ball targeted towards the ball target position
Vector2 ShotController::getPlaceBehindBall(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();
    Vector2 preferredShotVector = ball->pos - shotTarget;
    double distanceBehindBall = 2.0*Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    return ball->pos + preferredShotVector.stretchToLength(distanceBehindBall);
}

// use Numtree GTP to go to a place behind the ball
ShotData ShotController::goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition,
        std::pair<Vector2, Vector2> line) {
    auto ball = world::world->getBall();
    control::PosVelAngle pva = robot.getNumtreePosControl()->getPosVelAngle(std::make_shared<world::Robot>(robot),
            robotTargetPosition);
    //TODO: if (rotating to this angle from current angle will hit ball) then pva.angle=angle towards ball
    if ((robot.pos - robotTargetPosition).length() < 0.3) {
        pva.angle = (line.second - line.first).toAngle();
    }

    ShotData shotData(pva);
    return shotData;
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
ShotData ShotController::moveStraightToBall(world::Robot robot, std::pair<Vector2, Vector2> lineToDriveOver) {
    control::PosVelAngle pva = robot.getBasicPosControl()->getPosVelAngle(std::make_shared<world::Robot>(robot),
            lineToDriveOver.second);
    pva.angle = (lineToDriveOver.second - lineToDriveOver.first).angle();
    ShotData shotData(pva);
    return shotData;
}

/// Now we should have the ball and kick it.
ShotData ShotController::shoot(world::Robot robot, std::pair<Vector2, Vector2> driveLine, Vector2 shotTarget, bool chip,
        BallSpeed desiredBallSpeed) {

    auto ball = world::world->getBall();

    // move towards the ball
    control::PosVelAngle pva = robot.getBasicPosControl()->getPosVelAngle(std::make_shared<world::Robot>(robot),
            driveLine.second);
    pva.angle = (driveLine.second - driveLine.first).angle();

    ShotData shotData(pva);

    // set the kicker and kickforce
    if (chip) {
        shotData.chip = true;
        shotData.kick = false;

        // TODO calibrate chip speed
        shotData.kickSpeed = determineKickForce(ball->pos.dist(shotTarget), desiredBallSpeed);
    }
    else {
        shotData.chip = false;
        shotData.kick = true;
        shotData.kickSpeed = determineKickForce(ball->pos.dist(shotTarget), desiredBallSpeed);
    }
    return shotData;
}

/// Determine how fast we should kick for a pass at a given distance
double ShotController::determineKickForce(double distance, BallSpeed desiredBallSpeed) {
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();

    double velocity = 0;
    switch (desiredBallSpeed) {
    case DRIBBLE_KICK:velocity = sqrt(distance)*rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5);
        break;
    case LAY_STILL_AT_POSITION:velocity = sqrt(distance)*rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5);
        break;
    case PASS:
        velocity = distance > maxPowerDist ? rtt::ai::Constants::MAX_KICK_POWER() : sqrt(distance)
                *
                        rtt::ai::Constants::MAX_KICK_POWER()/
                sqrt(maxPowerDist)*
                1.2;
        break;
    case MAX_SPEED:velocity = rtt::ai::Constants::MAX_KICK_POWER();
        break;
    }

    // limit the output to the max kick speed
    return std::min(velocity, rtt::ai::Constants::MAX_KICK_POWER());
}

void ShotController::makeCommand(ShotData data, roboteam_msgs::RobotCommand &command) {
    command.x_vel = data.vel.x;
    command.y_vel = data.vel.y;
    command.w = data.angle.getAngle();
    command.chipper = data.chip;
    command.chipper_vel = data.kickSpeed;
    command.kicker = data.kick;
    command.kicker_forced = data.forced;
    command.kicker_vel = data.kickSpeed;
    command.geneva_state = data.genevaState;
}

bool ShotController::robotAngleIsGood(world::Robot &robot,
        std::pair<Vector2, Vector2> lineToDriveOver, ShotPrecision precision) {
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
// we rotate this vector according to the angle with the shotline
// we intentionally need to remove ball pos because we are rotating the vector
Vector2
ShotController::getPlaceBehindBallForGenevaState(world::Robot robot, Vector2 shotTarget, int genevaState) {
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

    // move the ball left or right {2,1,0,-1,-2} cm for genevastates {1,2,3,4,5}
    double moveLength = (3-genevaState) * 0.01;

    // get the angle perpendicular to the line, then shift the line by the moveLength
    Angle angle = (line.second - line.first).toAngle() + M_PI_2;
    shiftedLine.first += angle.toVector2(moveLength);
    shiftedLine.second += angle.toVector2(moveLength);
    return shiftedLine;
}

int ShotController::determineOptimalGenevaState(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();

    // determine the shortest position from where to kick the ball
    Vector2 robotToBall = ball->pos - robot.pos;
    Vector2 preferredShotVector = shotTarget - ball->pos;

    // determine the angle between the robot position and the shotline
    Angle angleWithShotline = robotToBall.toAngle() - preferredShotVector.toAngle();
    if (angleWithShotline.getAngle() > toRadians(20)) {
        return 2;
    }
    else if (angleWithShotline.getAngle() < (- toRadians(20))) {
        return 4;
    }
    return 3;
}
// should only be called if geneva is not turning
ShotData ShotController::moveAndShootGrSim(world::Robot robot, bool chip,
        std::pair<Vector2, Vector2> lineToDriveOver, BallSpeed desiredBallSpeed) {
    ShotData shotData;
    bool hasBall = world::world->ourRobotHasBall(robot.id, Constants::MAX_KICK_RANGE());
    if (hasBall) {
        std::cout<<"SHOOTING"<<std::endl;
        shotData = shoot(robot, lineToDriveOver, aimTarget, chip, desiredBallSpeed);
    }
    else {
        std::cout<<"MOVING TO BALL"<<std::endl;
        shotData = moveStraightToBall(robot, lineToDriveOver);
    }
    shotData.forced = true;
    return shotData;

}
ShotData ShotController::moveAndShoot(rtt::ai::world::Robot robot, bool chip,
        std::pair<Vector2, Vector2> lineToDriveOver, BallSpeed desiredBallSpeed) {
    control::PosVelAngle pva = robot.getBasicPosControl()->getPosVelAngle(std::make_shared<world::Robot>(robot),
            lineToDriveOver.second);
    pva.angle = (lineToDriveOver.second - lineToDriveOver.first).toAngle();
    ShotData shotData(pva);
    auto ball = world::world->getBall();
    if (chip) {
        shotData.chip = true;
        shotData.kick = false;
        // TODO calibrate chip speed
        shotData.kickSpeed = determineKickForce(ball->pos.dist(aimTarget), desiredBallSpeed);
    }
    else {
        shotData.chip = false;
        shotData.kick = true;
        shotData.kickSpeed = determineKickForce(ball->pos.dist(aimTarget), desiredBallSpeed);
    }
    shotData.forced = false;
    return shotData;
}

}// control
} // ai
} // rtt