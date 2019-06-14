//
// Created by thijs on 17-12-18.
//

#ifndef ROBOTEAM_AI_ATTACK_H
#define ROBOTEAM_AI_ATTACK_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Attack : public Skill {
public:
    explicit Attack(string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_ATTACK_H
