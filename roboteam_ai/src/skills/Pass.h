//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include <roboteam_ai/src/control/numTrees/NumTreePosControl.h>
#include <roboteam_ai/src/control/BasicPosControl.h>
#include "Skill.h"
#include <roboteam_ai/src/coach/pass/PassCoach.h>
#include <roboteam_ai/src/control/PositionUtils.h>
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/shotControllers/ShotController.h>

namespace rtt {
namespace ai {

class Pass : public Skill {
protected:
    const double CLOSE_ENOUGH_TO_BALL = 0.7;
    bool chip = false;
    int fails = 0;
    int failsUntilChip = -1;
    bool passInitialized = false;
    bool hasShot = false;
    RobotPtr robotToPassTo;
    Vector2 targetPos;
    virtual void initiatePass();
    bool didShootProperly();
    int robotToPassToID = -1;
    Vector2 getKicker();


public:
    explicit Pass(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

} //ai
} //rtt


#endif //ROBOTEAM_AI_PASS_H
