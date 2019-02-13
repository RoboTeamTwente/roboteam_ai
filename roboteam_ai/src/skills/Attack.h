//
// Created by thijs on 17-12-18.
//

#ifndef ROBOTEAM_AI_ATTACK_H
#define ROBOTEAM_AI_ATTACK_H

#include <roboteam_ai/src/control/ControlGoToPos.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class Attack : public Skill {
    private:
        control::ControlGoToPos goToPos;
        Vector2 deltaPos;
        Vector2 targetPos;
        int genevaState = 0;

    public:
        explicit Attack(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_ATTACK_H
