//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_PASSCOACH_H
#define ROBOTEAM_AI_PASSCOACH_H

#include <chrono>
#include <world/Robot.h>

namespace rtt {
namespace ai {
namespace coach {

class PassCoach {
public:
    using Robot = world::Robot;
    using RobotPtr = std::shared_ptr<Robot>;

    PassCoach();
    void resetPass(int robotID);
    int initiatePass(int passerID);
    bool isReadyToReceivePass();
    void setReadyToReceivePass(bool readyToReceivePass);
    int getRobotBeingPassedTo();
    void setRobotBeingPassedTo(int robotBeingPassedTo);
    bool isPassed();
    void setPassed(bool passed);

    virtual int determineReceiver(int passerID);
    bool passTakesTooLong();
    void updatePassProgression();
    bool validReceiver(const RobotPtr& passer, const RobotPtr& receiver, bool freeKick = false);

private:

    const double RECEIVER_MAX_DISTANCE_INTO_OUR_SIDE = 0.2;

    const double SMALLEST_MIN_PASS_DISTANCE = 10 * Constants::ROBOT_RADIUS();

    std::chrono::time_point<std::chrono::steady_clock> passStartTime;
    std::chrono::time_point<std::chrono::steady_clock> receiveStartTime;

    bool passTimerStarted = false;
    bool receiveTimerStarted = false;

    const double MAX_PASS_TIME = 8.0; //seconds
    const double MAX_RECEIVE_TIME = 5.0; //seconds

    bool readyToReceivePass{};
    int robotPassing = -1;
public:
    int getRobotPassing() const;

private:
    int robotBeingPassedTo = -1;
    bool passed{};
};

extern PassCoach g_pass;

} // coach
} // ai
} // rtt
#endif //ROBOTEAM_AI_PASSCOACH_H
