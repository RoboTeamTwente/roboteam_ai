//
// Created by thijs on 1-4-19.
//

#include "Robot.h"
#include "World.h"
#include "Ball.h"

#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {
namespace world {

Robot::Robot(const roboteam_msgs::WorldRobot &copy, Team team, unsigned char genevaState, unsigned long worldNumber)
        :distanceToBall(- 1.0), iHaveBall(false), genevaState(genevaState), lastUpdatedWorldNumber(worldNumber),
         id(copy.id), angle(copy.angle), pos(copy.pos), vel(copy.vel), angularVelocity(copy.w), team(team) {

    if (id != - 1) {
        workingGeneva = Constants::ROBOT_HAS_WORKING_GENEVA(id);
    }
}

Robot::Robot()
        :distanceToBall(- 1.0), iHaveBall(false), genevaState(0), lastUpdatedWorldNumber(0),
         id(- 1), angle(- 1.0), angularVelocity(- 1.0), team(invalid) {
}

const roboteam_msgs::WorldRobot Robot::toMessage() const {
    roboteam_msgs::WorldRobot robotMsg;
    robotMsg.angle = static_cast<float>(angle);
    robotMsg.w = static_cast<float>(angularVelocity);
    robotMsg.pos = pos;
    robotMsg.vel = vel;
    robotMsg.id = static_cast<unsigned int>(id);
    return robotMsg;
}

bool Robot::hasBall(double maxDist) {
    return iHaveBall && distanceToBall < maxDist && distanceToBall >= 0.0;
}

double Robot::getDistanceToBall() {
    return distanceToBall;
}

void Robot::updateRobot(const roboteam_msgs::WorldRobot &robotMsg, const Ball &ball, unsigned long worldNumber) {
    if (robotMsg.id == this->id) {
        this->pos = robotMsg.pos;
        this->vel = robotMsg.vel;
        this->angle = robotMsg.angle;
        this->angularVelocity = robotMsg.w;
        this->lastUpdatedWorldNumber = worldNumber;
    }
    distanceToBall = findBallDistance(ball.pos);
    iHaveBall = distanceToBall >= 0.0;
}

double Robot::findBallDistance(const Vector2 &ballPos) {

    // the angles of the left/right side of the dribbler
    Angle leftSideAngle = angle - Constants::DRIBBLER_ANGLE_OFFSET();
    Angle rightSideAngle = angle + Constants::DRIBBLER_ANGLE_OFFSET();

    // the positions of the left/right side of the  dribbler
    Vector2 leftSideOfDribbler = pos + leftSideAngle.toVector2(Constants::ROBOT_RADIUS());
    Vector2 rightSideOfDribbler = pos + rightSideAngle.toVector2(Constants::ROBOT_RADIUS());

    // if the ball is in the triangle defined by the left/right side of the dribbler and the middle of the robot
    if (control::ControlUtils::pointInTriangle(ballPos, pos, leftSideOfDribbler, rightSideOfDribbler)) {
        return 0.0;
    }

    // points that define a rectangle in front of the dribbler of the robot
    Vector2 leftSideMaxPoint = leftSideOfDribbler + angle.toVector2(Constants::MAX_BALL_BOUNCE_RANGE());
    Vector2 rightSideMaxPoint = rightSideOfDribbler + angle.toVector2(Constants::MAX_BALL_BOUNCE_RANGE());

    // if the ball is in the rectangle defined by the left/right side of the dribbler and two points in front of these
    if (control::ControlUtils::pointInRectangle(ballPos, leftSideOfDribbler, rightSideOfDribbler,
            leftSideMaxPoint, rightSideMaxPoint)) {

        return control::ControlUtils::distanceToLine(ballPos, leftSideOfDribbler, rightSideOfDribbler);
    }

    // if the robot does not have ball
    return - 1.0;
}

unsigned char Robot::getGenevaState() const {
    return genevaState;
}

void Robot::setGenevaState(unsigned char state) {

    if (state < 0 || state > 5) {
        std::cout << "setting invalid geneva state (" << (int)state <<
        ") for robot with id " << id << std::endl;
    }
    else if (! workingGeneva) {
        std::cout << "setting geneva state (" << (int)state <<
        ") for robot without working geneva with id " << id << std::endl;
    }
    else if (state != 0) {
        previousGenevaState = genevaState;
        genevaState = state;
        timeGenevaChanged = world::world->getTime();
    }
}

const unsigned long &Robot::getLastUpdatedWorldNumber() const {
    return lastUpdatedWorldNumber;
}

bool Robot::isGenevaReady() const {
    return world->getTime() - timeGenevaChanged > abs(genevaState - previousGenevaState)*timeToChangeOneGenevaState;
}

bool Robot::hasWorkingGeneva() const {
    return workingGeneva;
}

} //world
} //ai
} //rtt