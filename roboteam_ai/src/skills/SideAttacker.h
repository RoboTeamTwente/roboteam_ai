//
// Created by thijs on 9-1-19.
//

#ifndef ROBOTEAM_AI_SIDEATTACKER_H
#define ROBOTEAM_AI_SIDEATTACKER_H

#include "Skill.h"
#include <roboteam_ai/src/coach/OffensiveCoach.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>

namespace rtt {
namespace ai {

class SideAttacker : public Skill {
    private:

    control::NumTreePosControl goToPos;
    Vector2 deltaPos;
    Vector2 targetPos;

    int zone;
    static int robotsInMemory;

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
