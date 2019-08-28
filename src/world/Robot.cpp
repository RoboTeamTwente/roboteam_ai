//
// Created by thijs on 1-4-19.
//

#include "include/roboteam_ai/world/Robot.h"
#include "include/roboteam_ai/world/World.h"
#include "include/roboteam_ai/world/Ball.h"

#include "include/roboteam_ai/control/ControlUtils.h"
#include "include/roboteam_ai/control/shotControllers/ShotController.h"
#include "include/roboteam_ai/control/ballHandling/BallHandlePosControl.h"
#include "include/roboteam_ai/control/numTrees/NumTreePosControl.h"
#include "include/roboteam_ai/control/BasicPosControl.h"

namespace rtt {
namespace ai {
namespace world {

Robot::Robot(const roboteam_proto::WorldRobot &copy, Team team,
        unsigned char genevaState, unsigned char dribblerState, unsigned long worldNumber)
        :pidPreviousVel(Vector2()), distanceToBall(- 1.0), iHaveBall(false), lastUpdatedWorldNumber(worldNumber),
         genevaState(genevaState), dribblerState(dribblerState), id(copy.id()),
         angle(copy.angle()), pos(copy.pos()), vel(copy.vel()), angularVelocity(copy.w()), team(team) {

    if (id > - 1 && id < 16) {
        workingGeneva = Constants::ROBOT_HAS_WORKING_GENEVA(id);
        workingDribbler = Constants::ROBOT_HAS_WORKING_DRIBBLER(id);
        workingBallSensor = Constants::ROBOT_HAS_WORKING_BALL_SENSOR(id);
    }
    else {
        std::cout << "Warning: creating robot with id = " << id << "!" << std::endl;
        workingGeneva = false;
        workingDribbler = false;
        workingBallSensor = false;
    }

    // set up control controllers
    shotController = std::make_shared<control::ShotController>();
    numTreePosControl = std::make_shared<control::NumTreePosControl>();
    basicPosControl = std::make_shared<control::BasicPosControl>();
    ballHandlePosControl = std::make_shared<control::BallHandlePosControl>();
}

Robot::Robot()
        :distanceToBall(- 1.0), iHaveBall(false), lastUpdatedWorldNumber(0), genevaState(0), workingGeneva(false),
         dribblerState(0), workingDribbler(false), workingBallSensor(false),
         id(- 1), angle(- 1.0), angularVelocity(- 1.0), team(invalid) {

    shotController = nullptr;
    numTreePosControl = nullptr;
    basicPosControl = nullptr;
    ballHandlePosControl = nullptr;
}

bool Robot::hasBall(double maxDist) const {
    return iHaveBall && distanceToBall < maxDist && distanceToBall >= 0.0;
}

double Robot::getDistanceToBall() {
    return distanceToBall;
}

void Robot::updateRobot(const roboteam_proto::WorldRobot &robotMsg, const BallPtr &ball, unsigned long worldNumber) {
    if (static_cast<int>(robotMsg.id()) == this->id) {
        this->pos = robotMsg.pos();
        this->vel = robotMsg.vel();
        this->angle = robotMsg.angle();
        this->angularVelocity = robotMsg.w();
        this->lastUpdatedWorldNumber = worldNumber;
    }
    distanceToBall = calculateDistanceToBall(ball->pos);
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

int Robot::getGenevaState() const {
    return genevaState;
}

/*
 * Check if the geneva state is in the range [0,5]
 * and that the robot has a working geneva.
 */
bool Robot::genevaStateIsValid(int state) {
    // if the state is invalid
    if (state < 0 || state > 5) {
        std::cout << "Invalid geneva state (" << (int) state << ") for robot with id " << id << std::endl;
        return false;
    }

    // if the geneva does not work

    if (! workingGeneva) {
        if (genevaStateIsDifferent(state)) {
            std::cout << "Trying to set geneva state (" << (int) state << ") for robot without working geneva with id "
                      << id << std::endl;
        }
        return false;
    }

    return true;
}

/*
 * Returns true when the geneva state is different
 */
bool Robot::genevaStateIsDifferent(int state) {
    return !(state == genevaState || state == 0);
}

/*
 * Specific robotfeedback handling.
 * If the received state from feedback is valid and different we override our internal geneva state with our expectations from reality.
 */
void Robot::setGenevaStateFromFeedback(int state) {
    if (!genevaStateIsValid(state)) return;
    if (!genevaStateIsDifferent(state)) return;

    previousGenevaState = genevaState;
    genevaState = state;
    timeGenevaChanged = world::world->getTime();
}

void Robot::setGenevaState(int state) {
    if (!genevaStateIsValid(state)) return;
    if (!genevaStateIsDifferent(state)) return;

    // if the geneva is turning currently
    if (! isGenevaReady()) {
        std::cout << "The geneva is not ready yet. for robot with id " << id << std::endl;
        std::cout << "still turning for " << world->getTime() - timeGenevaChanged << " s" << std::endl;
        std::cout << "turning from " << genevaState << " to " << state << std::endl;

        return;
    }

    previousGenevaState = genevaState;
    genevaState = state;
    timeGenevaChanged = world::world->getTime();
}

bool Robot::isGenevaReady() const {
    return world->getTime() - timeGenevaChanged >=
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
        std::cout << "setting invalid dribbler state (" << (int) dribbler <<
                  ") for robot with id " << id << std::endl;
    }
    else if (! workingDribbler && dribbler != 0) {
        std::cout << "setting dribbler state (" << (int) dribbler <<
                  ") for robot without working dribbler with id " << id << std::endl;
    }
    else {
        if (previousDribblerState != dribblerState) {
            auto maxStatesChanged = static_cast<int>(
                    (world::world->getTime() - timeDribblerChanged)/timeToChangeOneDribblerLevel);
            auto statesChanged = std::max(abs(previousDribblerState - dribblerState), maxStatesChanged);
            previousDribblerState = previousDribblerState +
                    abs(previousDribblerState - dribblerState)/(previousDribblerState - dribblerState)*statesChanged;
        }
        dribblerState = dribbler;
        timeDribblerChanged = world::world->getTime();
    }
}

bool Robot::isDribblerReady() const {
    return (world::world->getTime() - timeDribblerChanged) >
            abs(dribblerState - previousDribblerState)*timeToChangeOneDribblerLevel;
}

bool Robot::hasWorkingDribbler() const {
    return workingDribbler;
}

const std::shared_ptr<control::ShotController> &Robot::getShotController() const {
    return shotController;
}

const std::shared_ptr<control::NumTreePosControl> &Robot::getNumtreePosControl() const {
    return numTreePosControl;
}

const std::shared_ptr<control::BasicPosControl> &Robot::getBasicPosControl() const {
    return basicPosControl;
}

const std::shared_ptr<control::BallHandlePosControl> &Robot::getBallHandlePosControl() const {
    return ballHandlePosControl;
}

void Robot::setWorkingGeneva(bool genevaIsWorking) {
    workingGeneva = genevaIsWorking;
}

const Vector2 &Robot::getPidPreviousVel() const {
    return pidPreviousVel;
}

void Robot::setPidPreviousVel(const Vector2 &pidVel) {
    pidPreviousVel = pidVel;
}


void Robot::resetShotController() {
    shotController = std::make_shared<control::ShotController>();
}

void Robot::resetNumTreePosControl() {
    numTreePosControl = std::make_shared<control::NumTreePosControl>();
}

void Robot::resetBasicPosControl() {
    basicPosControl = std::make_shared<control::BasicPosControl>();
}

void Robot::resetBallHandlePosControl() {
    ballHandlePosControl = std::make_shared<control::BallHandlePosControl>();
}

bool Robot::hasWorkingBallSensor() const {
    return workingBallSensor;
}

void Robot::setHasWorkingBallSensor(bool hasWorkingBallSensor) {
    workingBallSensor = hasWorkingBallSensor;
}

void Robot::setTimeToChangeOneGenevaState(double timeToChangeOneGenevaState) {
    Robot::timeToChangeOneGenevaState = timeToChangeOneGenevaState;
}

bool Robot::isBatteryLow() const {
    return batteryLow;
}

void Robot::setBatteryLow(bool batteryLow) {
    Robot::batteryLow = batteryLow;
}

/*
 * Returns true when a feedback packet has been detected at most 2 second ago.
 */
bool Robot::hasRecentFeedback() {
    return (lastReceivedFeedbackMoment > world->getTime() - 1.0);
}

/*
 * indicate that we got feedback at this moment.
 */
void Robot::UpdateFeedbackReceivedTime() {
    lastReceivedFeedbackMoment = world->getTime();
}

Vector2 Robot::getKicker() const {
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT() + 0.1, 0};
    return this->pos + distanceToKicker.rotate(this->angle);
}

} //world
} //ai
} //rtt