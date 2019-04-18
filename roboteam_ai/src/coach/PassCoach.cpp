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
    std::cout << "Reset pass!" << std::endl;
    passed = false;
    readyToReceivePass = false;
    robotBeingPassedTo  = -1;
    timerStarted = false;
}

int PassCoach::initiatePass(int passerID) {
//    if (robotBeingPassedTo != -1) {
//        if(!passTakesTooLong()) {
//            return -1;
//        }
//    }
    std::cout << "Initialize pass!" << std::endl;
    resetPass();

    robotBeingPassedTo = determineReceiver(passerID);
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
        start = std::chrono::system_clock::now();
        timerStarted = true;
    }
}

bool PassCoach::passTakesTooLong() {
    if (timerStarted) {
        auto now = chrono::system_clock::now();
        double elapsedSeconds = chrono::duration_cast<chrono::seconds>(now - start).count();

        if(elapsedSeconds > MAX_PASS_TIME ) {
            return true;
        }
    }
}

} // coach
} // ai
} // rtt
