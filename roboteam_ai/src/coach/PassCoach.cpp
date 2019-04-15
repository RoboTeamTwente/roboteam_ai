//
// Created by mrlukasbos on 19-3-19.
//

#include "PassCoach.h"
#include "../world/Field.h"

namespace rtt {
namespace ai {
namespace coach {

PassCoach g_pass;

void PassCoach::resetPass() {
    passed = false;
    readyToReceivePass = false;
    robotBeingPassedTo  = -1;
}

int PassCoach::initiatePass() {
    resetPass();

    robotBeingPassedTo = determineReceiver();
    return robotBeingPassedTo;
}

bool PassCoach::isReadyToReceivePass() {
    return readyToReceivePass;
}

void PassCoach::setReadyToReceivePass(bool readyToReceivePass) {
    this->readyToReceivePass = readyToReceivePass;
}

int PassCoach::getRobotBeingPassedTo() {
    return robotBeingPassedTo;
}

void PassCoach::setRobotBeingPassedTo(int robotBeingPassedTo) {
    this->robotBeingPassedTo = robotBeingPassedTo;
}

bool PassCoach::isPassed() {
    return passed;
}

const Vector2 &PassCoach::getPassPosition() const {
    return passPosition;
}

int PassCoach::determineReceiver() {
    //TODO: Add filters like offensive, midfield, etc...
    double bestScore = 0;
    int bestRobotID = -1;
    for(auto &robot : world::world->getUs()) {
        double score = coach::CoachHeuristics::calculatePassScore(robot.pos);
        if (score > bestScore) {
            bestScore = score;
            bestRobotID = robot.id;
        }
    }

    return bestRobotID;
}

void PassCoach::setPassed(bool passed) {
this->passed = passed;
}

} // coach
} // ai
} // rtt
