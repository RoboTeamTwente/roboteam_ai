//
// Created by mrlukasbos on 19-3-19.
//

#include "PassCoach.h"
#include "../world/Field.h"

namespace rtt {
namespace ai {
namespace coach {

double PassCoach::MIN_PASS_DISTANCE;

PassCoach g_pass;

PassCoach::PassCoach() {
    auto field = world::field->get_field();
    double goalWidth = field.goal_width;
    MIN_PASS_DISTANCE = std::max(goalWidth / 2, SMALLEST_MIN_PASS_DISTANCE);
}

void PassCoach::resetPass() {
    passed = false;
    readyToReceivePass = false;
    robotPassing = -1;
    robotBeingPassedTo  = -1;

    passTimerStarted = false;
    receiveTimerStarted = false;
}

int PassCoach::initiatePass(int passerID) {
    // Check whether a pass is already in progress that is not taking too long yet
    if (robotBeingPassedTo != -1) {
        if(!passTakesTooLong()) {
            return -1;
        }
    }
    resetPass();

    passStartTime = std::chrono::steady_clock::now();
    passTimerStarted = true;

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
    auto passer = world::world->getRobotForId(passerID, true);
    for(auto &robot : world::world->getUs()) {
        if(robot.id == passerID) continue;
        if(!checkIfValidReceiver(passerID, robot.id)) continue;
        double score = passScore.calculatePassScore(robot.pos);
        if (score > bestScore) {
            bestScore = score;
            bestRobotID = robot.id;
        }
    }

    if (bestRobotID == -1) {
        std::cerr << "Help" << std::endl;
    }

    return bestRobotID;
}

void PassCoach::setPassed(bool passed) {
    this->passed = passed;
    if(passed) {
        receiveStartTime = std::chrono::steady_clock::now();
        receiveTimerStarted = true;
        passTimerStarted = false;
    }
}

bool PassCoach::passTakesTooLong() {
    if (passTimerStarted && !passed) {
        auto now = chrono::steady_clock::now();
        double elapsedSeconds = chrono::duration_cast<chrono::seconds>(now - passStartTime).count();
        if (elapsedSeconds > MAX_PASS_TIME) {
            std::cout << "Pass takes too long!" << std::endl;
            return true;
        }
    }

    if (receiveTimerStarted) {
        auto now = chrono::steady_clock::now();
        double elapsedSeconds = chrono::duration_cast<chrono::seconds>(now - receiveStartTime).count();
        if (elapsedSeconds > MAX_RECEIVE_TIME) {
            std::cout << "Receive takes too long!" << std::endl;
            return true;
        }
    }

    return false;
}

int PassCoach::getRobotPassing() const {
    return robotPassing;
}
void PassCoach::updatePassProgression() {
    if (robotBeingPassedTo != -1) {
        if (passTakesTooLong()) {
            resetPass();
        }
    }

}
bool PassCoach::checkIfValidReceiver(int passerId, int receiverId) {
    auto passer = world::world->getRobotForId(passerId, true);
    auto receiver = world::world->getRobotForId(receiverId, true);
    if (receiverId == robotDealer::RobotDealer::getKeeperID() || receiverId == passerId) {
        std::cout << "ERROR" << std::endl;
        return false;
    }

    if (receiver->pos.x < -RECEIVER_MAX_DISTANCE_INTO_OUR_SIDE) {
        return false;
    }

    return true;

    //return (passer->pos - receiver->pos).length() < MIN_PASS_DISTANCE;

}

} // coach
} // ai
} // rtt
