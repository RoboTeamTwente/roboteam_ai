//
// Created by baris on 8-4-19.
//

#ifndef ROBOTEAM_AI_PASSIVESTOP_H
#define ROBOTEAM_AI_PASSIVESTOP_H
#include "Skill.h"
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>

namespace rtt{
namespace ai {
class PassiveStop : public Skill {
    public:
        explicit PassiveStop(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
    private:
        Vector2 targetPos;
        control::NumTreePosControl goToPos;
        Vector2 getTargetPos();

};
}
}
#endif //ROBOTEAM_AI_PASSIVESTOP_H
