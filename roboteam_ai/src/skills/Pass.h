//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "Skill.h"
#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/control/PositionUtils.h>
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/shotControllers/ShotController.h>

namespace rtt {
namespace ai {

class Pass : public Skill {
private:

    const double CLOSE_ENOUGH_TO_BALL = 0.7;

    bool passInitialized = false;
    bool ballPlacement = false;
    RobotPtr robotToPassTo;

    Vector2 targetPos;
    int robotToPassToID = -1;

    std::shared_ptr<control::ShotController> shotControl;

    void initiatePass();
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
