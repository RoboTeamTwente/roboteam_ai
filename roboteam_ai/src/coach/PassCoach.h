//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_PASSCOACH_H
#define ROBOTEAM_AI_PASSCOACH_H

#include <roboteam_ai/src/coach/OffensiveCoach.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/src/coach/Ballplacement.h>

namespace rtt {
namespace ai {
namespace coach {

class PassCoach {
public:
    enum PassType {
        ballPlacement,
        offensive
    };

    PassType getPassType() const;
    void setPassType(PassType passType);

    coach::PassCoach::PassType stringToType(std::string string);

    void resetPass();
    int initiatePass(PassType type);

    bool isReadyToReceivePass();
    void setReadyToReceivePass(bool readyToReceivePass);

    int getRobotBeingPassedTo();
    void setRobotBeingPassedTo(int robotBeingPassedTo);

    bool isPassed();
    void setPassed(bool passed);

    const Vector2 &getPassPosition() const;
    void setPassPosition(const Vector2 &passPosition);

private:
    bool readyToReceivePass;
    int robotBeingPassedTo;
    bool passed;
    PassType passType;
    Vector2 passPosition;
public:
};

extern PassCoach g_pass;

} // coach
} // ai
} // rtt
#endif //ROBOTEAM_AI_PASSCOACH_H
