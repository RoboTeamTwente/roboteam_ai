//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "Skill.h"
#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include <roboteam_ai/src/utilities/Constants.h>

namespace rtt {
namespace ai {

class Pass : public Skill {
private:
    bool ballPlacement = false;
    RobotPtr robotToPassTo;

    Vector2 targetPos;
    int robotToPassToID = -1;
    control::NumTreePosControl numTreeGtp = control::NumTreePosControl(true, true, true);
    control::BasicPosControl basicGtp = control::BasicPosControl (false, true, true);

    Status getBall();
    Status moveBehindBall(Vector2 behindBallPos);
    Status shoot();

    double determineKickForce(double distance);
    Vector2 getKicker();

public:
    explicit Pass(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void determineRobotToPassTo();
};

} //ai
} //rtt


#endif //ROBOTEAM_AI_PASS_H
