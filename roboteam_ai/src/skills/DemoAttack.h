//
// Created by thijs on 17-12-18.
//

#ifndef ROBOTEAM_AI_DEMOATTACK_H
#define ROBOTEAM_AI_DEMOATTACK_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class DemoAttack : public Skill {
private:

    const double BEHIND_BALL_CHECK = 0.6;
    const double BEHIND_BALL_TARGET = 0.4;
    const double SWITCH_TO_BASICGTP_DISTANCE = 0.10;

    Vector2 deltaPos;
    Vector2 targetPos;
    bool ownGoal = false;
    bool shot = false;
public:
    explicit DemoAttack(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_DEMOATTACK_H
