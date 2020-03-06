//
// Created by mrlukasbos on 19-3-19.
//

#include "coach/PassCoach.h"
#include <chrono>
#include "coach/heuristics/PassScore.h"
#include "utilities/RobotDealer.h"
#include "world/FieldComputations.h"

namespace rtt::ai::coach {

PassCoach g_pass;

PassCoach::PassCoach() {}

void PassCoach::resetPass(int robotID) {
    if (robotID == robotBeingPassedTo || robotID == robotPassing || robotID == -1) {
        passed = false;
        readyToReceivePass = false;
        robotPassing = -1;
        robotBeingPassedTo = -1;

        passTimerStarted = false;
        receiveTimerStarted = false;
    }
}

int PassCoach::initiatePass(const Field &field, int passerID) {
    // Check whether a pass is already in progress that is not taking too long yet
    if (robotBeingPassedTo != -1) {
        if (!passTakesTooLong()) {
            return -1;
        }
    }
    resetPass(-1);

    passStartTime = std::chrono::steady_clock::now();
    passTimerStarted = true;

    robotBeingPassedTo = determineReceiver(field, passerID);
    if (robotBeingPassedTo == -1) {
        resetPass(-1);
        return -1;
    }

    robotPassing = passerID;
    return robotBeingPassedTo;
}

bool PassCoach::isReadyToReceivePass() { return readyToReceivePass; }

void PassCoach::setReadyToReceivePass(bool readyToReceivePass) { this->readyToReceivePass = readyToReceivePass; }

int PassCoach::getRobotBeingPassedTo() { return robotBeingPassedTo; }

void PassCoach::setRobotBeingPassedTo(int robotBeingPassedTo) { this->robotBeingPassedTo = robotBeingPassedTo; }

bool PassCoach::isPassed() { return passed; }

int PassCoach::determineReceiver(const Field &field, int passerID) {
    coach::PassScore passScore;
    double bestScore = 0;
    int bestRobotID = -1;
    auto passer = world_new::World::instance()->getWorld()->getRobotForId(passerID, true);

    for (auto &robot : world_new::World::instance()->getWorld()->getUs()) {
        if (!validReceiver(field, passer.value(), robot)) continue;
        double score = passScore.calculatePassScore(field, robot->getPos());
        if (score > bestScore) {
            bestScore = score;
            bestRobotID = robot->getId();
        }
    }

    return bestRobotID;
}

void PassCoach::setPassed(bool passed) {
    this->passed = passed;
    if (passed) {
        receiveStartTime = std::chrono::steady_clock::now();
        receiveTimerStarted = true;
        passTimerStarted = false;
    }
}

bool PassCoach::passTakesTooLong() {
    if (passTimerStarted && !passed) {
        auto now = std::chrono::steady_clock::now();
        double elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - passStartTime).count();
        if (elapsedSeconds > MAX_PASS_TIME) {
            return true;
        }
    }

    if (receiveTimerStarted) {
        auto now = std::chrono::steady_clock::now();
        double elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - receiveStartTime).count();
        if (elapsedSeconds > MAX_RECEIVE_TIME) {
            return true;
        }
    }

    return false;
}

int PassCoach::getRobotPassing() const { return robotPassing; }

void PassCoach::updatePassProgression() {
    if (robotBeingPassedTo != -1) {
        if (passTakesTooLong()) {
            resetPass(-1);
        }
    }
}

bool PassCoach::validReceiver(const Field &field, const world_new::view::RobotView &passer, const world_new::view::RobotView &receiver, bool freeKick) {
        auto ball = world_new::World::instance()->getWorld()->getBall();

        if (!ball || !passer || !receiver) {
            return false;
        }
        if (receiver->getId() == robotDealer::RobotDealer::getKeeperID() || receiver->getId() == passer->getId()) {
            return false;
        }

        if (!freeKick) {
            if (receiver->getPos().x < -RECEIVER_MAX_DISTANCE_INTO_OUR_SIDE) {
                return false;
            }
            auto MIN_PASS_DISTANCE = std::max((double)field.getGoalWidth() / 2, SMALLEST_MIN_PASS_DISTANCE);
            if ((passer->getPos() - receiver->getPos()).length() < MIN_PASS_DISTANCE) {
                return false;
            }
        }

        return true;
}

}  // namespace rtt::ai::coach
