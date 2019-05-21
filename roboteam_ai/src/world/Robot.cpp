//
// Created by thijs on 1-4-19.
//

#include "Robot.h"
#include "World.h"
#include "Ball.h"

#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/control/shotControllers/ShotController.h>

namespace rtt {
namespace ai {
namespace world {

Robot::Robot(const roboteam_msgs::WorldRobot &copy, Team team,
        unsigned char genevaState, unsigned char dribblerState, unsigned long worldNumber)
        :distanceToBall(- 1.0), iHaveBall(false), lastUpdatedWorldNumber(worldNumber), genevaState(genevaState),
         dribblerState(dribblerState),
         id(copy.id), angle(copy.angle), pos(copy.pos), vel(copy.vel), angularVelocity(copy.w), team(team) {

    if (id > - 1 && id < 16) {
        workingGeneva = Constants::ROBOT_HAS_WORKING_GENEVA(id);
        workingDribbler = Constants::ROBOT_HAS_WORKING_DRIBBLER(id);
    }
    else {
        workingGeneva = false;
        workingDribbler = false;
    }

    // set up control controllers
    shotController = std::make_shared<control::ShotController>();
    numtreeGTP = std::make_shared<control::NumTreePosControl>();
    basicGTP = std::make_shared<control::BasicPosControl>();
}

Robot::Robot()
        :distanceToBall(- 1.0), iHaveBall(false), lastUpdatedWorldNumber(0), genevaState(0), workingGeneva(false),
        dribblerState(0), workingDribbler(false),
        id(- 1), angle(- 1.0), angularVelocity(- 1.0), team(invalid) {
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
    distanceToBall = calculateDistanceToBall(ball.pos);
    iHaveBall = distanceToBall >= 0.0;
}

double Robot::calculateDistanceToBall(const Vector2 &ballPos) {

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

const unsigned long Robot::getLastUpdatedWorldNumber() const {
    return lastUpdatedWorldNumber;
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

bool Robot::isGenevaReady() const {
    return world->getTime() - timeGenevaChanged >
    abs(genevaState - previousGenevaState)*timeToChangeOneGenevaState;
}

bool Robot::hasWorkingGeneva() const {
    return workingGeneva;
}

unsigned char Robot::getDribblerState() const {
    return dribblerState;
}

void Robot::setDribblerState(unsigned char dribbler) {

    if (dribbler < 0 || dribbler > 31) {
        std::cout << "setting invalid dribbler state (" << (int)dribbler <<
                  ") for robot with id " << id << std::endl;
    }
    else if (! workingDribbler) {
        std::cout << "setting dribbler state (" << (int)dribbler <<
                  ") for robot without working dribbler with id " << id << std::endl;
    }
    else {
        previousDribblerState = dribblerState;
        dribblerState = dribbler;
        timeDribblerChanged = world::world->getTime();
    }
}

bool Robot::isDribblerReady() const {
    return world->getTime() - timeDribblerChanged >
            abs(dribblerState - previousDribblerState)*timeToChangeOneDribblerLevel;
}

bool Robot::hasWorkingDribbler() const {
    return workingDribbler;
}

const shared_ptr<control::ShotController> &Robot::getShotController() const {
    return shotController;
}

const shared_ptr<control::NumTreePosControl> &Robot::getNumtreeGtp() const {
    return numtreeGTP;
}

const shared_ptr<control::BasicPosControl> &Robot::getBasicGtp() const {
    return basicGTP;
}

} //world
} //ai
} //rtt