//
// Created by thijs on 17-12-18.
//

#ifndef ROBOTEAM_AI_ATTACK_H
#define ROBOTEAM_AI_ATTACK_H

#include <control/PosController.h>
#include <control/shotControllers/ShotController.h>
#include <include/roboteam_ai/coach/OffensiveCoach.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class Attack : public Skill {
private:
    rtt::ai::coach::OffensiveCoach offensiveCoach;
public:
    explicit Attack(string name, bt::Blackboard::Ptr blackboard, rtt::ai::coach::OffensiveCoach offensiveCoach);
    explicit Attack(string name, bt::Blackboard::Ptr blackboard);
    rtt::ai::coach::OffensiveCoach getOffensiveCoach();

    Status onUpdate() override;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_ATTACK_H
