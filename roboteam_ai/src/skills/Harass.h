//
// Created by baris on 12-12-18.
//

#ifndef ROBOTEAM_AI_HARASS_H
#define ROBOTEAM_AI_HARASS_H

#include <roboteam_ai/src/control/PositionManager.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class Harass : public Skill {

    public:
        explicit Harass(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
    private:
        int harassmentTarget = - 1;
        void pickHarassmentTarget();
        bool harassBallOwner = false;
        control::PositionManager goToPos;
};

}
}
#endif //ROBOTEAM_AI_HARASS_H
