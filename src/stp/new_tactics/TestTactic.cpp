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

Status TestTactic::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
    // Check if the skills are all finished
    if(skills.finished()) {
        return Status::Success;
    }

    // Update the current skill with the new SkillInfo
    return skills.update(info);
}

void TestTactic::onTerminate() noexcept {
    // Call terminate on all skills
    for(auto &x : skills) {
        x->terminate();
    }
}

} // namespace rtt::ai::stp

