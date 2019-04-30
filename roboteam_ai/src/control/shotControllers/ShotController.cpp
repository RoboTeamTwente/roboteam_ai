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

    // always set genevaIsTurning to true if it was 'long ago'.
    if (ros::Time::now().toSec() > (lastTimeGenevaChanged + secondsToTurnGeneva)) {
      genevaIsTurning = false;
    }

    determineGenevaAndPosition(robot, shotTarget);

    // check the properties
    bool isOnLineToBall = onLineToBall(robot, ball, behindBallPosition, currentDesiredGeneva);
    bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(0.80, shotTarget, robot.pos, 0.3);

   ShotData shotData;
   if (isOnLineToBall && isBehindBall) {
       bool hasBall = world::world->ourRobotHasBall(robot.id, Constants::MAX_KICK_RANGE());
       shotData = hasBall && !genevaIsTurning ? shoot(robot, shotTarget) : moveStraightToBall(robot, currentDesiredGeneva);
   } else {
       shotData = goToPlaceBehindBall(robot, behindBallPosition, currentDesiredGeneva);
   }

   // Make sure the Geneva state is always correct
   shotData.genevaState = currentDesiredGeneva;
   return shotData;
}


void ShotController::determineGenevaAndPosition(const world::Robot &robot, const Vector2 &shotTarget) {// determine the position for the robot to stand and the corresponding geneva angle
    // only change values if we don't turn the geneva (and are thus able to turn it)
    if (useAutoGeneva && !genevaIsTurning && robot.hasWorkingGeneva) {
        auto oldGenevaState = currentDesiredGeneva;

        auto positionAndGeneva = getGenevaPlaceBehindBall(robot, shotTarget);
        behindBallPosition = positionAndGeneva.first;
        currentDesiredGeneva = positionAndGeneva.second;

        int genevaDifference = abs(oldGenevaState - currentDesiredGeneva);

        if (genevaDifference != 0) {
            genevaIsTurning = true;
            // each turn should increase the time which the geneva is turning
            secondsToTurnGeneva = genevaDifference * 0.5;
            lastTimeGenevaChanged = ros::Time::now().toSec();
        }
    } else if (!useAutoGeneva || !robot.hasWorkingGeneva) {
        behindBallPosition = getPlaceBehindBall(robot, shotTarget);
        currentDesiredGeneva = 3;
    }
}

/// check if a robot is on a line to a ball
bool ShotController::onLineToBall(const world::Robot &robot, const world::World::BallPtr &ball, const Vector2 &behindBallPosition, int genevaState) {
    Vector2 lineStart = behindBallPosition; // behindBallposition is already compensated for geneva offset
    Vector2 lineEnd = getGenevaLineOffsetPoint(ball->pos, genevaState);

    if (precision == HIGH) {
        return ControlUtils::distanceToLine(robot.pos, lineStart, lineEnd) < 0.001;
    } else if (precision == MEDIUM) {
        return ControlUtils::distanceToLine(robot.pos, lineStart, lineEnd) < 0.005;
    }
    return ControlUtils::distanceToLine(robot.pos, lineStart, lineEnd) < 0.01;
}

/// return the place behind the ball targeted towards the ball target position
Vector2 ShotController::getPlaceBehindBall(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();
    Vector2 preferredShotVector = ball->pos - shotTarget;
    double distanceBehindBall = 2.0*Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    return ball->pos + preferredShotVector.stretchToLength(distanceBehindBall);
}

// use Numtree GTP to go to a place behind the ball
ShotData ShotController::goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition, int genevaState) {
    auto ball = world::world->getBall();

    control::PosVelAngle pva = numTreeGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), robotTargetPosition);

    if (robot.pos.dist(robotTargetPosition) < 0.3) {
        pva.angle = (getGenevaLineOffsetPoint(ball->pos, genevaState) - robot.pos).angle();
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

    Vector2 desiredTarget = getGenevaLineOffsetPoint(ball->pos + placeBehindBallVector, desiredGeneva);
    return std::make_pair(desiredTarget, desiredGeneva);
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
ShotData ShotController::moveStraightToBall(world::Robot robot, int genevaState) {
    auto ball = world::world->getBall();
    Vector2 targetPos = getGenevaLineOffsetPoint(ball->pos, genevaState);

    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot),  targetPos);
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

    double velocity = 0;
    switch(ballSpeed) {
        case DRIBBLE_KICK:
            velocity = sqrt(distance) *rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5);
            break;
        case LAY_STILL_AT_POSITION:
            velocity = sqrt(distance) *rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5) ;
            break;
        case PASS:
            velocity = distance > maxPowerDist ? rtt::ai::Constants::MAX_KICK_POWER() : sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER()/sqrt(maxPowerDist)*1.2;
            break;
        case MAX_SPEED:
            velocity = rtt::ai::Constants::MAX_KICK_POWER();
            break;
    }

    // limit the output to the max kick speed
    return std::min(velocity, rtt::ai::Constants::MAX_KICK_POWER());
}
void ShotController::makeCommand(ShotData data, roboteam_msgs::RobotCommand &command) {

    command.x_vel = data.vel.x;
    command.y_vel = data.vel.y;
    command.w = data.angle.getAngle();
    command.kicker = data.kick;
    command.kicker_forced = data.kick;
    command.kicker_vel = data.kickSpeed;
    command.geneva_state = data.genevaState;

}


Vector2 ShotController::getGenevaLineOffsetPoint(Vector2 point, int genevaState) {
    if (genevaState == 3) {
        return point;
    }
    std::map<int, double> genevaLineOffset;
    genevaLineOffset[1] = Constants::GRSIM() ? 0.01 : 0.02;
    genevaLineOffset[2] = Constants::GRSIM() ? 0.005 : 0.01;
    genevaLineOffset[4] = Constants::GRSIM() ? - 0.005 : -0.01;
    genevaLineOffset[5] = Constants::GRSIM() ? - 0.01 : -0.02;
    point = point + point.rotate(M_PI_2).stretchToLength(genevaLineOffset[genevaState]);
    return point;
}

} // control
} // ai
} // rtt