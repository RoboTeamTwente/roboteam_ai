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
    setPassed(false);
    setReadyToReceivePass(false);
    setRobotBeingPassedTo(-1);
}

int PassCoach::initiatePass() {
    resetPass();

    // TODO: More logic to decide which robot to pass to. Possibly split initiate in initiate and findRobotToPassTo
    int robotBeingPassedTo = g_offensiveCoach.getBestStrikerID();
    setRobotBeingPassedTo(robotBeingPassedTo);
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

void PassCoach::setPassed(bool passed) {
    this->passed = passed;
}

} // coach
} // ai
} // rtt
