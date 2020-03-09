//
// Created by roboteam on 9/3/20.
//

#include <stp/new_skills/Rotate.h>
#include <stp/new_skills/SetDribbler.h>
#include "stp/new_tactics/TestTactic.h"


namespace rtt::ai::stp {

void TestTactic::onInitialize() noexcept {
    skills = rtt::collections::state_machine<Skill, Status, SkillInfo>{Rotate(), SetDribbler(), Rotate()};

    // Initialize first skill
    skills.initialize();
}

void TestTactic::onUpdate(Status const &status) noexcept {

}

void TestTactic::onTerminate() noexcept {
    // Call terminate on all skills
    for(auto &x : skills) {
        x->terminate();
    }
}

SkillInfo calculateInfoForSkill(TacticInfo const &info) noexcept {
    SkillInfo skillInfo;

    /// Update the skillInfo parameters here

    return skillInfo;
}

} // namespace rtt::ai::stp

