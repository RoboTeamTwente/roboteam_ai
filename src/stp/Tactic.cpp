//
// Created by jessevw on 03.03.20.
//

#include "include/roboteam_ai/stp/Tactic.h"
#include "stp/Skill.h"
namespace rtt::ai::stp {

    void Tactic::updateActiveSkill(TacticInfo tacticInfo) {
        std::find_if(skills.begin(), skills.end(), [] (const Skill& s) {return s.getStatus() == Status::Success;});

    }
}
