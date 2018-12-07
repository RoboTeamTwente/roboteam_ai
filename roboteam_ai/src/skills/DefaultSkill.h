//
// Created by thijs on 19-11-18.
//

#ifndef ROBOTEAM_AI_DEFAULTSKILL_H
#define ROBOTEAM_AI_DEFAULTSKILL_H

#include "Skill.h"

namespace rtt {
namespace ai {

class DefaultSkill : public Skill {
    private:
        bool variable1;
    public:
        explicit DefaultSkill(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;
        Status update() override;
        void initialize() override;
};
} // ai
} // rtt

#endif //ROBOTEAM_AI_DEFAULTSKILL_H
