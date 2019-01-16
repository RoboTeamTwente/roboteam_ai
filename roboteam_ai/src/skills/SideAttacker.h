//
// Created by thijs on 9-1-19.
//

#ifndef ROBOTEAM_AI_SIDEATTACKER_H
#define ROBOTEAM_AI_SIDEATTACKER_H

#include "Skill.h"

namespace rtt {
namespace ai {

class SideAttacker : public Skill {
    private:


        control::ControlGoToPos goToPos;
        control::ControlKick kicker;
        Vector2 deltaPos;
        Vector2 targetPos;

    public:
        explicit SideAttacker(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SIDEATTACKER_H
