//
// Created by mrlukasbos on 6-12-18.
//

#ifndef ROBOTEAM_AI_HALT_H
#define ROBOTEAM_AI_HALT_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Halt : public Skill {
    public:
        explicit Halt(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_HALT_H
