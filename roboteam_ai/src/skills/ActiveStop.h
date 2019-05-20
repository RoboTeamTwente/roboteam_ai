//
// Created by baris on 8-4-19.
//

#ifndef ROBOTEAM_AI_ACTIVESTOP_H
#define ROBOTEAM_AI_ACTIVESTOP_H
#include "Skill.h"
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>

namespace rtt{
namespace ai {


class ActiveStop : public Skill {
    public:
        explicit ActiveStop(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
    private:
        Vector2 targetPos;
        static int attack;
        bool attacker = false;
        static Vector2 getOffensiveActivePoint();
        static Vector2 getDefensiveActivePoint();

};
}
}
#endif //ROBOTEAM_AI_ACTIVESTOP_H