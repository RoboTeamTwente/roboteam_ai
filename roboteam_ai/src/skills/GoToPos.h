//
// Created by baris on 24/10/18.
//

#ifndef ROBOTEAM_AI_GOTOPOS_H
#define ROBOTEAM_AI_GOTOPOS_H

#include "Skill.h"

namespace rtt {
namespace ai {

class GoToPos : public Skill {
    public:
        Vector2 targetPos = {0, 0};
        double maxVel = Constants::DEFAULT_MAX_VEL();
        control::PositionManager goToPos;
        double errorMargin = Constants::GOTOPOS_ERROR_MARGIN();

        explicit GoToPos(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
};
} // ai
} // rtt

#endif //ROBOTEAM_AI_GOTOPOS_H
