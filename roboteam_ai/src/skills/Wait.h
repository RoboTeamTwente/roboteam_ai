//
// Created by robzelluf on 6/5/19.
//

#ifndef ROBOTEAM_AI_WAIT_H
#define ROBOTEAM_AI_WAIT_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Wait : public Skill {
private:
    Angle lockedAngle;
    int tick = 0;
    int ticks = 0;
public:
    explicit Wait(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
};

}
}

#endif //ROBOTEAM_AI_WAIT_H
