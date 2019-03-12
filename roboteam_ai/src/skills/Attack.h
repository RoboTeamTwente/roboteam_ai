//
// Created by thijs on 17-12-18.
//

#ifndef ROBOTEAM_AI_ATTACK_H
#define ROBOTEAM_AI_ATTACK_H

#include <roboteam_ai/src/control/PositionController.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class Attack : public Skill {
    private:
        control::PositionController goToPos;
        Vector2 deltaPos;
        Vector2 targetPos;
        bool ownGoal = false;
        bool shot = false;
    public:
        explicit Attack(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_ATTACK_H
