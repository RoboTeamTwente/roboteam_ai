//
// Created by mrlukasbos on 19-3-19.
//

#include "coach/PassCoach.h"
#include "coach/heuristics/PassScore.h"
#include "utilities/RobotDealer.h"
#include "world/Field.h"
#include <chrono>
#include <world/World.h>

namespace rtt::ai::coach {

    PassCoach g_pass;

    void PassCoach::resetPass(int robotID) {
        if (!(robotID == robotBeingPassedTo || robotID == robotPassing || robotID == -1)) {
            return;
        }
        passed = false;
        readyToReceivePass = false;
        robotPassing = -1;
        robotBeingPassedTo = -1;

        passTimerStarted = false;
        receiveTimerStarted = false;
    }

    int PassCoach::initiatePass(int passerID) {
        // Check whether a pass is already in progress that is not taking too long yet
        if (robotBeingPassedTo != -1 && !passTakesTooLong()) {
            return -1;
        }
        resetPass(-1);

        passStartTime = std::chrono::steady_clock::now();
        passTimerStarted = true;

        robotBeingPassedTo = determineReceiver(passerID);
        if (robotBeingPassedTo == -1) {
            resetPass(-1);
            return -1;
        }

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
        coach::PassScore passScore{ };
        double bestScore = 0;
        int bestRobotID = -1;
        auto passer = world::world->getRobotForId(passerID, true);
        for (auto &robot : world::world->getUs()) {
            if (!validReceiver(passer, robot)) continue;
            double score = passScore.calculatePassScore(robot->pos);
            if (score > bestScore) {
                bestScore = score;
                bestRobotID = robot->id;
            }
        }

        return bestRobotID;
    }

    void PassCoach::setPassed(bool passed) {
        this->passed = passed;
        if (!passed) {
            return;
        }
        receiveStartTime = std::chrono::steady_clock::now();
        receiveTimerStarted = true;
        passTimerStarted = false;

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

    int PassCoach::getRobotPassing() const {
        return robotPassing;
    }

    void PassCoach::updatePassProgression() {
        if (robotBeingPassedTo == -1) {
            return;
        }

        if (!passTakesTooLong()) {
            return;
        }

        resetPass(-1);
    }

    bool PassCoach::validReceiver(const RobotPtr &passer, const RobotPtr &receiver, bool freeKick) {
        auto ball = world::world->getBall();

        if (!ball || !passer || !receiver) {
            return false;
        }
        if (receiver->id == robotDealer::RobotDealer::getKeeperID() || receiver->id == passer->id) {
            return false;
        }

        if (!freeKick) {
            if (receiver->pos.x < -RECEIVER_MAX_DISTANCE_INTO_OUR_SIDE) {
                return false;
            }

            const auto minPassDistance = std::max(world::field->get_field().goal_width() / 2.0,
                                                  SMALLEST_MIN_PASS_DISTANCE);
            if ((passer->pos - receiver->pos).length() < minPassDistance) {
                return false;
            }
        }

        return true;
    }

} // rtt::ai::coach
