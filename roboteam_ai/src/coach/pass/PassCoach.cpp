//
// Created by mrlukasbos on 19-3-19.
//

#include "PassCoach.h"
#include "roboteam_ai/src/world/Field.h"
#include "Pass.h"

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

void PassCoach::resetPass(int robotID) {
   currentPass = nullptr;
}

int PassCoach::initiatePass(int passerID) {
    Pass newPass;


    robotBeingPassedTo = determineReceiver(passerID);
    if (robotBeingPassedTo == -1) {
        resetPass(-1);
        return -1;
    }

    robotPassing = passerID;
    return robotBeingPassedTo;
}

int PassCoach::determineReceiver(int passerID) {
    coach::PassScore passScore;
    double bestScore = 0;
    int bestRobotID = -1;
    auto passer = world::world->getRobotForId(passerID, true);
    for(auto &robot : world::world->getUs()) {
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
            return true;
        }
    }

    if (receiveTimerStarted) {
        auto now = chrono::steady_clock::now();
        double elapsedSeconds = chrono::duration_cast<chrono::seconds>(now - receiveStartTime).count();
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
    if (robotBeingPassedTo != -1) {
        if (passTakesTooLong()) {
            resetPass(-1);
        }
    }

}
bool PassCoach::validReceiver(RobotPtr passer, RobotPtr receiver) {
    auto ball = world::world->getBall();
    if (receiver->id == robotDealer::RobotDealer::getKeeperID() || receiver->id == passer->id) {
        return false;
    }

    if (receiver->pos.x < -RECEIVER_MAX_DISTANCE_INTO_OUR_SIDE) {
        return false;
    }

    if((passer->pos - receiver->pos).length() < MIN_PASS_DISTANCE) {
        return false;
    }

    if(receiver->pos.x - ball->pos.x < 0) {
        return false;
    }

    return true;
}

} // coach
} // ai
} // rtt
