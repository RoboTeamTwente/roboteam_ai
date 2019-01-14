//
// Created by baris on 5-12-18.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Pass : public Skill {
    private:
        control::ControlGoToPos goToPos;
        Vector2 deltaPos;
        Vector2 targetPos;

        bool amIClosest;
        bool newTarget;
        bool defensive;
        int robotToPass;
    public:
        explicit Pass(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
};
} // ai
} // rtt


#endif //ROBOTEAM_AI_PASS_H
