//
// Created by mrlukasbos on 24-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include "ShotController.h"


namespace rtt {
namespace ai {
namespace control {

ShotController::ShotController(shotPrecision precision, bool useAutoGeneva)
: precision(precision), useAutoGeneva(useAutoGeneva) { }


ShotData ShotController::getShotData(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::worldObj.getBall();

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

    //
    bool isBehindBall = coach::g_generalPositionCoach.isRobotBehindBallToPosition(0.30, robotToPassTo->pos, robot.pos);



    // if at position
        // drive a little bit forward and shoot

    // else
        // go to position
        // if already close, already turn towards ball




//    if (ballIsMovingFast && ballIsShotTowardsReceiver) {
//        coach::g_pass.setPassed(true);
//        return Status::Success;
//    } else if (isOnLineToBall && isBehindBall) {
//        return hasBall ? shoot() : getBall();
//    }

    ShotData shotData;


    return shotData;
}

Vector2 ShotController::getPlaceBehindBall(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::worldObj.getBall();
    Vector2 preferredShotVector = ball->pos - shotTarget;
    double distanceBehindBall = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    return robot.pos - preferredShotVector.stretchToLength(distanceBehindBall);
}

std::pair<Vector2, int> ShotController::getGenevePlaceBehindBall(world::Robot robot, Vector2 shotTarget) {

    // determine the shortest position from where to kick the ball
    auto ball = world::worldObj.getBall();
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
    } else if (angleWithShotline.getAngle() > control::ControlUtils::degreesToRadians(-5) {
        desiredGeneva = 2;
        placeBehindBallVector.rotate(control::ControlUtils::degreesToRadians(-10));
    }

    return std::make_pair(placeBehindBallVector, desiredGeneva);
}


/// Determine how fast we should kick for a pass at a given distance
//    double Pass::determineKickForce(double distance) {
//        const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();
//
//        // take square root of distance and scale it vertically such that the max kick force and max distance for max kick force are correct.
//        double kickSpeed = distance > maxPowerDist ? rtt::ai::Constants::MAX_KICK_POWER() : sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER()/sqrt(maxPowerDist) ;
//        return static_cast<float>(kickSpeed);
//    }



} // control
} // ai
} // rtt