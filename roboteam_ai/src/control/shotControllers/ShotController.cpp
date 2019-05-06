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
ShotController::ShotController(ShotPrecision precision, BallSpeed ballspeed, bool useAutoGeneva)
        :precision(precision), ballSpeed(ballspeed), useAutoGeneva(useAutoGeneva) {
    numTreeGtp.setAvoidBall(Constants::BALL_RADIUS() + Constants::ROBOT_RADIUS()*1.5);
    numTreeGtp.setCanMoveOutOfField(true);
    numTreeGtp.setCanMoveInDefenseArea(false);
}

/// return a ShotData (which contains data for robotcommands) for a specific robot to shoot at a specific target.
ShotData ShotController::getShotData(world::Robot robot, Vector2 shotTarget, bool chip) {
    auto ball = world::world->getBall();

    // always set genevaIsTurning to true if it was 'long ago'.
    if (ros::Time::now().toSec() > (lastTimeGenevaChanged + secondsToTurnGeneva)) {
        genevaIsTurning = false;
    }

    /*
     * Get the position where to stand and the geneva state we want to have there
     */
    determineGenevaAndPosition(robot, shotTarget, chip);
       behindBallPosition = ball->pos + relativeToBallPosition;
    /*
     * From that position we want to drive over a line towards the ball
     * This line has a small offset if we use geneva
     */
    std::pair<Vector2, Vector2> lineToDriveOver = std::make_pair(behindBallPosition, ball->pos);
  //  lineToDriveOver=offsetLine(lineToDriveOver,currentDesiredGeneva);


    // check the properties
    bool isOnLineToBall = onLineToBall(robot, lineToDriveOver);
    bool isBehindBall = control::PositionUtils::isRobotBehindBallToPosition(0.80, shotTarget, robot.pos, 0.3);
    bool validAngle = robotAngleIsGood(robot, lineToDriveOver);

//    Angle aim((lineToDriveOver.second - lineToDriveOver.first).angle());
//    if (isBehindBall&&!validAngle&&!isOnLineToBall){
//        std::cout<<robot.id<<": WAITING FOR BOTH: "<<"angdif: "<<abs(aim-robot.angle)<< " boxdif|||| "<<control::ControlUtils::distanceToLine(robot.pos,lineToDriveOver.first,lineToDriveOver.second)<<"  |DIST:"<<(robot.pos-ball->pos).length()<<std::endl;
//    }
//    if (validAngle&&isBehindBall&&!isOnLineToBall){
//        std::cout<<robot.id<<": WAITING FOR BOX: "<<" boxdif|||| "<<control::ControlUtils::distanceToLine(robot.pos,lineToDriveOver.first,lineToDriveOver.second)<<"  |DIST:"<<(robot.pos-ball->pos).length()<<std::endl;
//    }
//    if (isBehindBall&&isOnLineToBall&&!validAngle){
//        std::cout<<robot.id<<": WAITING FOR ANGLE: "<<"angdif: "<<abs(aim-robot.angle)<<"  |DIST:"<<(robot.pos-ball->pos).length()<<std::endl;
//    }

    ShotData shotData;
    if (isOnLineToBall && isBehindBall && validAngle) {
        std::cout << "my position is correct, only need to drive forward and/or shoot" << std::endl;
        bool hasBall = world::world->ourRobotHasBall(robot.id, Constants::MAX_KICK_RANGE());
        if (hasBall && ! genevaIsTurning) {
            shotData = shoot(robot, lineToDriveOver, shotTarget, chip);
        }
        else if (hasBall && genevaIsTurning) {
            // just stand still at the right angle
            shotData.vel = {0.0, 0.0};
            shotData.angle = (lineToDriveOver.second - lineToDriveOver.first).angle();
            std::cout << "Not shooting because geneva is turning for " << secondsToTurnGeneva << "s" << std::endl;
        } else {
            shotData = moveStraightToBall(robot, lineToDriveOver);
        }
    }
    else {
        shotData = goToPlaceBehindBall(robot, behindBallPosition, lineToDriveOver);
    }
    // Make sure the Geneva state is always correct
    shotData.genevaState = currentDesiredGeneva;
    std::cout << "geneva: " <<  currentDesiredGeneva << std::endl;
    return shotData;
}

void ShotController::determineGenevaAndPosition(const world::Robot &robot, const Vector2 &shotTarget,
        bool chip) {// determine the position for the robot to stand and the corresponding geneva angle
    auto ball = world::world->getBall();
    bool robotAlreadyVeryClose = robot.pos.dist(ball->pos) < 3.0*Constants::ROBOT_RADIUS();
    auto oldGenevaState = currentDesiredGeneva;

    // only change values if we don't turn the geneva (and are thus able to turn it)
    if (useAutoGeneva && ! genevaIsTurning && robot.hasWorkingGeneva &&  !robotAlreadyVeryClose && ! chip) {
        std::cout << "getting location for GENEVA shot" << std::endl;

        auto positionAndGeneva = getGenevaPlaceBehindBall(robot, shotTarget);
        relativeToBallPosition = positionAndGeneva.first;
        currentDesiredGeneva = positionAndGeneva.second;
        int genevaDifference = abs(oldGenevaState - currentDesiredGeneva);
        setGenevaDelay(genevaDifference);
    }
    else if (! useAutoGeneva || ! robot.hasWorkingGeneva  || chip ||  currentDesiredGeneva == -1) {
        std::cout << "getting location without geneva shot" << std::endl;

        bool shouldWait = true;
        if (currentDesiredGeneva == -1) shouldWait = false;
        relativeToBallPosition = getPlaceBehindBall(robot, shotTarget);
        currentDesiredGeneva = 3;
        int genevaDifference = abs(oldGenevaState - currentDesiredGeneva);
        if (shouldWait) setGenevaDelay(genevaDifference);
    }
}

void ShotController::setGenevaDelay(int genevaDifference) {
    if (genevaDifference != 0) {
        genevaIsTurning = true;
        // each turn should increase the time which the geneva is turning
        secondsToTurnGeneva = genevaDifference*0.3;
        lastTimeGenevaChanged = ros::Time::now().toSec();
    }
}

/// check if a robot is on a line to a ball

bool ShotController::onLineToBall(const world::Robot &robot, std::pair<Vector2, Vector2> line) {
    double dist = ControlUtils::distanceToLine(robot.pos, line.first, line.second);
    //std::cout<<dist<<" ";
    if (precision == HIGH) {
        return dist < 0.05;
    }
    else if (precision == MEDIUM) {
        return dist < 0.10;
    }
    return dist < 0.10;
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
    control::PosVelAngle pva = numTreeGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), robotTargetPosition);
    //TODO: if (rotating to this angle from current angle will hit ball) then pva.angle=angle towards ball
    if (robot.pos.dist(robotTargetPosition) < 0.3) {
        pva.angle = (line.second - line.first).angle();
    }

    ShotData shotData(pva);
    return shotData;
}

/// get a position behind the ball for a geneva shot towards the target
std::pair<Vector2, int> ShotController::getGenevaPlaceBehindBall(world::Robot robot, Vector2 shotTarget) {
    auto ball = world::world->getBall();

    // determine the shortest position from where to kick the ball
    Vector2 robotToBall = ball->pos - robot.pos;
    Vector2 preferredShotVector = shotTarget - ball->pos;

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
//    if (robotAngleWithLineToGoal > toRadians(15)) {
//        desiredGeneva = 1;
//        placeBehindBallVector = placeStraightBehindBallVector.rotate(toRadians(20));
//
//    } else
        if (robotAngleWithLineToGoal > toRadians(5)) {
        desiredGeneva = 2;
        placeBehindBallVector = placeStraightBehindBallVector.rotate(toRadians(10));

    }
    else if (robotAngleWithLineToGoal < - toRadians(5)) {
        desiredGeneva = 4;
        placeBehindBallVector = placeStraightBehindBallVector.rotate(- toRadians(10));

    }
//    else if (robotAngleWithLineToGoal < - toRadians(15)) {
//        desiredGeneva = 5;
//        placeBehindBallVector = placeStraightBehindBallVector.rotate(- toRadians(20));
//
//    }
    else {
        desiredGeneva = 3;
        placeBehindBallVector = placeStraightBehindBallVector;
    }
    return std::make_pair(placeBehindBallVector, desiredGeneva);
}

/// At this point we should be behind the ball. now we can move towards the ball to kick it.
ShotData ShotController::moveStraightToBall(world::Robot robot, std::pair<Vector2, Vector2> lineToDriveOver) {

    std::cout << "moving straight to ball" << std::endl;
    // we want to move to lineToDriveOver's end point (second)
    Vector2 projectedPos = robot.pos.project(lineToDriveOver.first, lineToDriveOver.second);

    Vector2 newDir=lineToDriveOver.second-projectedPos+(projectedPos-robot.pos)*2.0;
    control::PosVelAngle pva=basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot),lineToDriveOver.second);
    pva.vel=newDir.stretchToLength(pva.vel.length());
    /*
    Vector2 intermediateTarget = (projectedPos + lineToDriveOver.second)/2.0;
    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), intermediateTarget);
     */
    pva.angle = (lineToDriveOver.second - lineToDriveOver.first).angle();
    //std::cout << robot.id << " moving::" << robot.angle.getAngle() << " : cmd" << pva.angle.getAngle() << std::endl;
    ShotData shotData(pva);
    return shotData;
}

/// Now we should have the ball and kick it.
ShotData ShotController::shoot(world::Robot robot, std::pair<Vector2, Vector2> driveLine, Vector2 shotTarget,
        bool chip) {
    auto ball = world::world->getBall();

    // move towards the ball
    control::PosVelAngle pva = basicGtp.getPosVelAngle(std::make_shared<world::Robot>(robot), driveLine.second);
    pva.angle = (driveLine.second - driveLine.first).angle();
    //std::cout << robot.id << " shooting::" << abs(robot.angle.getAngle()-pva.angle.getAngle()) << std::endl;

    ShotData shotData(pva);

    // set the kicker and kickforce
    if (chip) {
        shotData.chip = true;
        shotData.kick = false;

        // TODO calibrate chip speed
        shotData.kickSpeed = determineKickForce(ball->pos.dist(shotTarget));
    }
    else {
        shotData.chip = false;
        shotData.kick = true;
        shotData.kickSpeed = determineKickForce(ball->pos.dist(shotTarget));
    }
    return shotData;
}

/// Determine how fast we should kick for a pass at a given distance
double ShotController::determineKickForce(double distance) {
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();

    double velocity = 0;
    switch (ballSpeed) {
    case DRIBBLE_KICK:velocity = sqrt(distance)*rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5);
        break;
    case LAY_STILL_AT_POSITION:velocity = sqrt(distance)*rtt::ai::Constants::MAX_KICK_POWER()/(sqrt(maxPowerDist)*1.5);
        break;
    case PASS:velocity = distance > maxPowerDist ? rtt::ai::Constants::MAX_KICK_POWER() : sqrt(distance)
                *rtt::ai::Constants::MAX_KICK_POWER()/sqrt(maxPowerDist)*1.2;
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
    command.kicker_forced = data.kick;
    command.kicker_vel = data.kickSpeed;
    command.geneva_state = data.genevaState;
}

bool ShotController::robotAngleIsGood(world::Robot &robot,
        std::pair<Vector2, Vector2> lineToDriveOver) {
    Angle aim((lineToDriveOver.second - lineToDriveOver.first).angle());
    double diff=abs(aim-robot.angle);
    if (precision==HIGH){
        return diff<0.05;
    }
    if (precision==MEDIUM) {
        return diff<0.25;
    }
    return diff<0.3;
}
std::pair<Vector2,Vector2> ShotController::offsetLine(std::pair<Vector2,Vector2> line, int genevaState){
    if (genevaState==3){
        return line;
    }
    std::pair<Vector2,Vector2> correctedLine;
    std::map<int, double> genevaLineOffset;
    genevaLineOffset[1] = Constants::GRSIM() ? 0.01 : 0.02;
    genevaLineOffset[2] = Constants::GRSIM() ? 0.005 : 0.01;
    genevaLineOffset[4] = Constants::GRSIM() ? - 0.005 : - 0.01;
    genevaLineOffset[5] = Constants::GRSIM() ? - 0.01 : - 0.02;
    double lineDir=(line.second-line.first).angle();
    Vector2 correction=Vector2(genevaLineOffset[genevaState],0).rotate(M_PI_2+lineDir);
    correctedLine.first=line.first+correction;
    correctedLine.second=line.second+correction;
    behindBallPosition=correctedLine.first;
    return correctedLine;


}

} // control
} // ai
} // rtt