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

int PassCoach::initiatePass(PassType type) {
    resetPass();

    // TODO: More logic to decide which robot to pass to. Possibly split initiate in initiate and findRobotToPassTo
    int robotBeingPassedTo;
    passType = type;
    if (passType == ballPlacement) {
        robotBeingPassedTo = robotDealer::RobotDealer::findRobotForRole("BallPlacementReceiver");
        passPosition = coach::g_ballPlacement.getBallPlacementPos();
    } else {
        robotBeingPassedTo = g_offensiveCoach.getBestStrikerID();
    }
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

PassCoach::PassType PassCoach::stringToType(std::string string) {
    if (string == "ballPlacement") {
        return coach::PassCoach::ballPlacement;
    }
    else {
        return coach::PassCoach::offensive;
    }
}

PassCoach::PassType PassCoach::getPassType() const {
    return passType;
}

void PassCoach::setPassType(PassCoach::PassType passType) {
    PassCoach::passType = passType;
}

const Vector2 &PassCoach::getPassPosition() const {
    return passPosition;
}

void PassCoach::setPassPosition(const Vector2 &passPosition) {
    PassCoach::passPosition = passPosition;
}

} // coach
} // ai
} // rtt
