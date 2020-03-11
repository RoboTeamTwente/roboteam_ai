//
// Created by roboteam on 9/3/20.
//

#include "stp/new_tactics/TestTactic.h"

#include <stp/new_skills/Rotate.h>
#include <stp/new_skills/GoToPos.h>

namespace rtt::ai::stp {

TestTactic::TestTactic(){
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, SkillInfo>{GoToPos(), Rotate()};
    skills.initialize();
}

void TestTactic::onInitialize() noexcept {



}

void TestTactic::onUpdate(Status const &status) noexcept {
}

void TestTactic::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

SkillInfo TestTactic::calculateInfoForSkill(TacticInfo const &info) noexcept {
    SkillInfo skillInfo;

    // TODO make this smarter and better
    skillInfo.setRobot(info.getRobot().value());
    skillInfo.setAngle(2.0);
    skillInfo.setDribblerSpeed(31);
    skillInfo.setTacticInfo(info);

    return skillInfo;
}

}  // namespace rtt::ai::stp
