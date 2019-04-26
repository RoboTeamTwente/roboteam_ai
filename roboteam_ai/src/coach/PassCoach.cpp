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
    robotPassing = -1;
    robotBeingPassedTo  = -1;
    timerStarted = false;
}

int PassCoach::initiatePass(int passerID) {
    // Check whether a pass is already in progress that is not taking too long yet
    if (robotBeingPassedTo != -1) {
        if(!passTakesTooLong()) {
            return -1;
        }
    }
    resetPass();

    robotBeingPassedTo = determineReceiver(passerID);
    robotPassing = passerID;
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

int PassCoach::determineReceiver(int passerID) {
    coach::PassScore passScore;
    double bestScore = 0;
    int bestRobotID = -1;
    for(auto &robot : world::world->getUs()) {
        if (robot.id == robotDealer::RobotDealer::getKeeperID() || robot.id == passerID) continue;
        if (robot.pos.x < -RECEIVER_MAX_DISTANCE_INTO_OUR_SIDE) continue;
        double score = passScore.calculatePassScore(robot.pos);
        if (score > bestScore) {
            bestScore = score;
            bestRobotID = robot.id;
        }
    }

    return bestRobotID;
}

void PassCoach::setPassed(bool passed) {
    this->passed = passed;
    if(passed) {
        start = std::chrono::steady_clock::now();
        timerStarted = true;
    }
}

bool PassCoach::passTakesTooLong() {
    if (timerStarted) {
        auto now = chrono::steady_clock::now();
        double elapsedSeconds = chrono::duration_cast<chrono::seconds>(now - start).count();

        return elapsedSeconds > MAX_PASS_TIME;
    }

    return false;
}

int PassCoach::getRobotPassing() const {
    return robotPassing;
}

} // coach
} // ai
} // rtt
