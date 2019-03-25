//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_PASSCOACH_H
#define ROBOTEAM_AI_PASSCOACH_H

#include <roboteam_ai/src/coach/OffensiveCoach.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>

namespace rtt {
namespace ai {
namespace coach {

class PassCoach {
public:
    PassCoach() = default;
    void resetPass();
    int initiatePass();

    bool isReadyToReceivePass();
    void setReadyToReceivePass(bool readyToReceivePass);

    int getRobotBeingPassedTo();
    void setRobotBeingPassedTo(int robotBeingPassedTo);

    bool isPassed();

    const Vector2 &getPassPosition() const;

    virtual int determineRobotToPassTo();

private:
    bool readyToReceivePass;
    int robotBeingPassedTo;
    bool passed;
    Vector2 passPosition;
};

extern PassCoach g_pass;

} // coach
} // ai
} // rtt
#endif //ROBOTEAM_AI_PASSCOACH_H
