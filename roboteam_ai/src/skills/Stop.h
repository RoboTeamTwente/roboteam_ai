//
// Created by baris on 3-4-19.
//

#ifndef ROBOTEAM_AI_STOP_H
#define ROBOTEAM_AI_STOP_H

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "Skill.h"
namespace rtt {
namespace ai {

class Stop : public Skill {
    public:
        explicit Stop(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
    private:
        bool isActive = false;
        Vector2 getActivePoint();
        control::NumTreePosControl goToPos;

};
}}

#endif //ROBOTEAM_AI_STOP_H
