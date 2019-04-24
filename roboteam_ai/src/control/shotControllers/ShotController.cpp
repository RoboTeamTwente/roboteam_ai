//
// Created by mrlukasbos on 24-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/control/PositionUtils.h>
#include "ShotController.h"


namespace rtt {
namespace ai {
namespace control {

ShotController::ShotController(shotPrecision precision, bool useAutoGeneva)
: precision(precision), useAutoGeneva(useAutoGeneva) {
   ball = world::worldObj.getBall();
}

ShotData ShotController::getShotData(world::Robot robot, Vector2 shotTarget) {
    int genevaState = 3;
    Vector2 behindBallPosition;

    // determine the position for the robot to stand and the corresponding geneva angle
    if (useAutoGeneva) {
        auto positionAndGeneva = getGenevePlaceBehindBall(robot, shotTarget);
        behindBallPosition = positionAndGeneva.first;
        genevaState = positionAndGeneva.second;
    } else {
        behindBallPosition = getPlaceBehindBall(robot, shotTarget);
        genevaState = 3;
    }

    // TODO implement precision here
    bool isOnLineToBall = control::ControlUtils::distanceToLine(robot.pos, ball->pos, behindBallPosition) < 0.0255;

    // TODO fix for 40 degrees so for wide geneva it will still be pretty crappy. 0.70 radians is 40 degrees
   bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(0.80, shotTarget, robot.pos);
   bool hasBall = world::world->ourRobotHasBall(robot.id, Constants::MAX_BALL_RANGE());

   ShotData shotData;
   if (isOnLineToBall && isBehindBall) {
       shotData = hasBall ? shoot(robot, shotTarget) : moveStraightToBall(robot);
   } else {
       shotData = goToPlaceBehindBall(robot, behindBallPosition);
   }

   // make sure the genevastate is always correct
   shotData.genevaState = genevaState;
   return shotData;
}

Vector2 ShotController::getPlaceBehindBall(world::Robot robot, Vector2 shotTarget) {
    Vector2 preferredShotVector = ball->pos - shotTarget;
    double distanceBehindBall = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    return robot.pos - preferredShotVector.stretchToLength(distanceBehindBall);
}

ShotData ShotController::goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition) {
    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), robotTargetPosition);
    ShotData shotData(pva);
    return shotData;
}


std::pair<Vector2, int> ShotController::getGenevePlaceBehindBall(world::Robot robot, Vector2 shotTarget) {

    // determine the shortest position from where to kick the ball
    Vector2 robotToBall = robot.pos - ball->pos;
    Vector2 preferredShotVector = ball->pos - shotTarget;

    // determine the angle between the robot position and the shotline
    Angle angleWithShotline = robotToBall.angle() - preferredShotVector.angle();

    // get the place behind the ball as if no geneva is used
    // we rotate this vector according to the angle with the shotline
    Vector2 placeBehindBallVector = robot.pos - getPlaceBehindBall(robot, shotTarget);

    int desiredGeneva = 3;
    if (angleWithShotline.getAngle() > control::ControlUtils::degreesToRadians(15)) {
        desiredGeneva = 5;
        placeBehindBallVector.rotate(control::ControlUtils::degreesToRadians(20));
    } else if (angleWithShotline.getAngle() > control::ControlUtils::degreesToRadians(5)) {
        desiredGeneva = 4;
        placeBehindBallVector.rotate(control::ControlUtils::degreesToRadians(10));
    } else if (angleWithShotline.getAngle() < control::ControlUtils::degreesToRadians(-15)) {
        desiredGeneva = 1;
        placeBehindBallVector.rotate(control::ControlUtils::degreesToRadians(-20));
    } else if (angleWithShotline.getAngle() > control::ControlUtils::degreesToRadians(-5)) {
        desiredGeneva = 2;
        placeBehindBallVector.rotate(control::ControlUtils::degreesToRadians(-10));
    }

    return std::make_pair(placeBehindBallVector, desiredGeneva);
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
ShotData ShotController::moveStraightToBall(world::Robot robot) {
    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot),  ball->pos);
    ShotData shotData(pva);
    return shotData;
}

/// Now we should have the ball and kick it.
ShotData ShotController::shoot(world::Robot robot, Vector2 shotTarget) {

    // move towards the ball
    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), shotTarget);
    ShotData shotData(pva);

    // set the kicker and kickforce
    shotData.kick = true;
    shotData.kickSpeed = determineKickForce(ball->pos.dist(shotTarget));

    return shotData;
}


// Determine how fast we should kick for a pass at a given distance
double ShotController::determineKickForce(double distance) {
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();

    // take square root of distance and scale it vertically such that the max kick force and max distance for max kick force are correct.
    double kickSpeed = distance > maxPowerDist ? rtt::ai::Constants::MAX_KICK_POWER() : sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER()/sqrt(maxPowerDist) ;
    return static_cast<float>(kickSpeed);
}



} // control
} // ai
} // rtt