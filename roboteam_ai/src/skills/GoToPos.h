//
// Created by baris on 24/10/18.
//

#ifndef ROBOTEAM_AI_GOTOPOS_H
#define ROBOTEAM_AI_GOTOPOS_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class GoToPos : public Skill {
    public:
        Vector2 targetPos = {0, 0};
        double maxAcc = Constants::MAX_ACC_LOWER();
        double maxVel = Constants::DEFAULT_MAX_VEL();
        double minVel = Constants::MIN_VEL();
        double errorMargin = Constants::GOTOPOS_ERROR_MARGIN();
        double prevVel = Vector2(robot->vel).length();

        control::NumTreePosControl gotopos;

        explicit GoToPos(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
};
} // ai
} // rtt

#endif //ROBOTEAM_AI_GOTOPOS_H
