//
// Created by thijs on 9-1-19.
//

#ifndef ROBOTEAM_AI_SIDEATTACKER_H
#define ROBOTEAM_AI_SIDEATTACKER_H

#include "Skill.h"
#include <include/roboteam_ai/coach/OffensiveCoach.h>
#include <include/roboteam_ai/control/numTrees/NumTreePosControl.h>
#include <roboteam_utils/Vector2.h>

namespace rtt {
namespace ai {

class SideAttacker : public Skill {
    private:
    Vector2 deltaPos;
    Vector2 targetPos;

    Vector2 getOffensivePosition();

    public:
        explicit SideAttacker(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SIDEATTACKER_H
