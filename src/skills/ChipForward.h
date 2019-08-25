//
// Created by robzelluf on 7/5/19.
//

#include "Skill.h"

#ifndef ROBOTEAM_AI_CHIPFORWARD_H
#define ROBOTEAM_AI_CHIPFORWARD_H

namespace rtt {
namespace ai {

class ChipForward : public Skill {
private:
    Vector2 aimPoint;
    bool hasChipped = false;
public:
    explicit ChipForward(string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
};

}
}


#endif //ROBOTEAM_AI_CHIPFORWARD_H
