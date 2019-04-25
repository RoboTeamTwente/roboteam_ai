//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_PASSCOACH_H
#define ROBOTEAM_AI_PASSCOACH_H

#include <roboteam_ai/src/coach/OffensiveCoach.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/src/coach/heuristics/PassScore.h>
#include <chrono>

namespace rtt {
namespace ai {
namespace coach {

class PassCoach {
public:
    PassCoach() = default;
    void resetPass();
    int initiatePass(int passerID);
    bool isReadyToReceivePass();
    void setReadyToReceivePass(bool readyToReceivePass);
    int getRobotBeingPassedTo();
    void setRobotBeingPassedTo(int robotBeingPassedTo);
    bool isPassed();
    void setPassed(bool passed);

    virtual int determineReceiver(int passerID);
    bool passTakesTooLong();

private:
    std::chrono::time_point<std::chrono::steady_clock> start;
    bool timerStarted = false;
    double MAX_PASS_TIME = 10.0; //seconds
    bool readyToReceivePass;
    int robotPassing = -1;
public:
    int getRobotPassing() const;

private:
    int robotBeingPassedTo = -1;
    bool passed;
};

extern PassCoach g_pass;

} // coach
} // ai
} // rtt
#endif //ROBOTEAM_AI_PASSCOACH_H
