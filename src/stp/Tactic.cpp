//
// Created by jessevw on 03.03.20.
//

#include "include/roboteam_ai/stp/Tactic.h"
#include "stp/Skill.h"
namespace rtt::ai::stp {

    void Tactic::updateActiveSkill(TacticInfo tacticInfo) {
        auto activeSkill = std::find_if(skills.begin(), skills.end(), [] (const Skill& s) {return s.getStatus() == Status::Running;});
        auto skillInfo = SkillInfo();
        auto tacticInfo = TacticInfo();
        tacticInfo.areasToAvoid = bla;
        tacticInfo.numberOfCandy = 5;
        tacticInfo.areasToAvoid
        tacticInfo.numberOfCandy == 9;
        activeSkill->update(skillInfo);
    }

    void Tactic::calculateInfoForSkill() {

    }
}
