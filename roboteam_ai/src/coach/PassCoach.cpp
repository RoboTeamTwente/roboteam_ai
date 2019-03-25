//
// Created by mrlukasbos on 19-3-19.
//

#include "PassCoach.h"
#include "../utilities/Field.h"

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

    robotBeingPassedTo = determineRobotToPassTo();
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

int PassCoach::determineRobotToPassTo() {
     return g_offensiveCoach.getBestStrikerID();
}
void PassCoach::setPassed(bool passed) {
this->passed = passed;
}

} // coach
} // ai
} // rtt
