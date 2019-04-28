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

ShotController::ShotController(ShotPrecision precision, BallSpeed ballspeed, bool useAutoGeneva)
: precision(precision), ballSpeed(ballspeed), useAutoGeneva(useAutoGeneva) {
    numTreeGtp.setAvoidBall();
    numTreeGtp.setCanMoveOutOfField(true);
    numTreeGtp.setCanMoveInDefenseArea(false);
}

/// return a ShotData (which contains data for robotcommands) for a specific robot to shoot at a specific target.
ShotData ShotController::getShotData(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();
    int genevaState = 3;
    Vector2 behindBallPosition;

    // determine the position for the robot to stand and the corresponding geneva angle
    if (useAutoGeneva) {
        auto positionAndGeneva = getGenevaPlaceBehindBall(robot, shotTarget);
        behindBallPosition = positionAndGeneva.first;
        genevaState = positionAndGeneva.second;
    } else {
        behindBallPosition = getPlaceBehindBall(robot, shotTarget);
        genevaState = 3;
    }

    // check the properties
    bool isOnLineToBall = onLineToBall(robot, ball, behindBallPosition);
    bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(0.80, shotTarget, robot.pos);

   ShotData shotData;
   if (isOnLineToBall && isBehindBall) {
       bool hasBall = world::world->ourRobotHasBall(robot.id, Constants::MAX_KICK_RANGE());
       shotData = hasBall ? shoot(robot, shotTarget) : moveStraightToBall(robot);
   } else {
       shotData = goToPlaceBehindBall(robot, behindBallPosition);
   }

   // Make sure the Geneva state is always correct
   shotData.genevaState = genevaState;
   return shotData;
}

/// check if a robot is on a line to a ball
bool ShotController::onLineToBall(const world::Robot &robot, const world::World::BallPtr &ball, const Vector2 &behindBallPosition) const {
    if (precision == HIGH) {
        return ControlUtils::distanceToLine(robot.pos, ball->pos, behindBallPosition) < 0.001;
    } else if (precision == MEDIUM) {
        return ControlUtils::distanceToLine(robot.pos, ball->pos, behindBallPosition) < 0.005;
    }
    return ControlUtils::distanceToLine(robot.pos, ball->pos, behindBallPosition) < 0.01;
}

/// return the place behind the ball targeted towards the ball target position
Vector2 ShotController::getPlaceBehindBall(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();
    Vector2 preferredShotVector = ball->pos - shotTarget;
    double distanceBehindBall = 2.0*Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    return ball->pos + preferredShotVector.stretchToLength(distanceBehindBall);
}

// use Numtree GTP to go to a place behind the ball
ShotData ShotController::goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition) {
    auto ball = world::world->getBall();

    control::PosVelAngle pva = numTreeGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), robotTargetPosition);

    if (robot.pos.dist(robotTargetPosition) < 0.3) {
        pva.angle = (ball->pos - robot.pos).angle();
    }

    ShotData shotData(pva);
    return shotData;
}

/// get a position behind the ball for a geneva shot towards the target
std::pair<Vector2, int> ShotController::getGenevaPlaceBehindBall(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();

    // determine the shortest position from where to kick the ball
    Vector2 robotToBall = ball->pos - robot.pos;
    Vector2 preferredShotVector =  shotTarget - ball->pos;

    // determine the angle between the robot position and the shotline
    Angle angleWithShotline = robotToBall.toAngle() - preferredShotVector.toAngle();

    // get the place behind the ball as if no geneva is used
    // we rotate this vector according to the angle with the shotline
    // we intentionally need to remove ball pos because we are rotating the vector
    Vector2 placeStraightBehindBallVector = getPlaceBehindBall(robot, shotTarget) - ball->pos;
    Vector2 placeBehindBallVector;

    double robotAngleWithLineToGoal = angleWithShotline.getAngle();
    int desiredGeneva;

    // handle geneva options
    if (robotAngleWithLineToGoal > toRadians(15)) {
        desiredGeneva = 1;
        placeBehindBallVector = placeStraightBehindBallVector.rotate(toRadians(20));

    } else if (robotAngleWithLineToGoal > toRadians(5)) {
        desiredGeneva = 2;
        placeBehindBallVector = placeStraightBehindBallVector.rotate(toRadians(10));

    } else if (robotAngleWithLineToGoal < -toRadians(5)) {
        desiredGeneva = 5;
        placeBehindBallVector = placeStraightBehindBallVector.rotate(-toRadians(20));

    } else if (robotAngleWithLineToGoal < -toRadians(15)) {
        desiredGeneva = 4;
        placeBehindBallVector = placeStraightBehindBallVector.rotate(-toRadians(10));

    } else {
        desiredGeneva = 3;
        placeBehindBallVector = placeStraightBehindBallVector;
    }


    return std::make_pair(ball->pos + placeBehindBallVector, desiredGeneva);
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
ShotData ShotController::moveStraightToBall(world::Robot robot) {
    auto ball = world::world->getBall();
    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot),  ball->pos);
    ShotData shotData(pva);
    return shotData;
}

/// Now we should have the ball and kick it.
ShotData ShotController::shoot(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();

    // move towards the ball
    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), shotTarget);
    ShotData shotData(pva);

    // set the kicker and kickforce
    shotData.kick = true;
    shotData.kickSpeed = determineKickForce(ball->pos.dist(shotTarget));

    return shotData;
}


/// Determine how fast we should kick for a pass at a given distance
double ShotController::determineKickForce(double distance) {
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();
    switch(ballSpeed) {
        case DRIBBLE_KICK:
            return sqrt(distance) *rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5) ;
        case LAY_STILL_AT_POSITION:
            return sqrt(distance) *rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5) ;
        case PASS:
            return distance > maxPowerDist ? rtt::ai::Constants::MAX_KICK_POWER() : sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER()/sqrt(maxPowerDist)*1.2 ;
        case MAX_SPEED:
            return rtt::ai::Constants::MAX_KICK_POWER();
    }
}
roboteam_msgs::RobotCommand ShotController::makeCommand(ShotData data) {
    roboteam_msgs::RobotCommand command;

    command.x_vel = data.vel.x;
    command.y_vel = data.vel.y;
    command.w = data.angle.getAngle();
    command.kicker = data.kick;
    command.kicker_forced = data.kick; // TODO this looks very meh
    command.kicker_vel = data.kickSpeed;
    command.geneva_state = data.genevaState;


    return command;
}

} // control
} // ai
} // rtt