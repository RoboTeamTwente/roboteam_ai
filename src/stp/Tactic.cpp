//
// Created by jessevw on 03.03.20.
//

#include "stp/Tactic.h"

namespace rtt::ai::stp {

void Tactic::initialize() noexcept { onInitialize(); }

Status Tactic::update(StpInfo const &info) noexcept {
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info")
        currentStatus = Status::Failure;
        return Status::Failure;
    }

    // Update skill info
    auto skill_info = calculateInfoForSkill(info);

    if(skill_info) {
        // Update the current skill with the new SkillInfo
        auto status = skills.update(skill_info.value());

        // Call onUpdate on a skill for specific behaviour
        onUpdate(status);
    }
    else {
        RTT_ERROR("Not all data was present, bad update!")
    }

    // Check if the skills are all finished
    if (skills.finished() || forceTacticSuccess(skill_info.value())) {
        if (!isEndTactic()) {
            currentStatus = Status::Success;
            return Status::Success;
        }
        // Make sure we keep executing the last tactic since it is an end tactic
        skills.reset();
        currentStatus = Status::Waiting;
        return Status::Waiting;
    }

    // the tactic will not be reset if it's the first skill
    if (skills.current_num() != 0 && shouldTacticReset(skill_info.value())) {
        reset();
    }

    // if the failing condition is true, the current tactic will fail
    if (skill_info && isTacticFailing(skill_info.value())) {
        currentStatus = Status::Failure;
        return Status::Failure;
    }

    currentStatus = Status::Running;
    return Status::Running;
}

void Tactic::terminate() noexcept { onTerminate(); }

void Tactic::reset() noexcept { skills.reset(); }

Skill *Tactic::getCurrentSkill() {
    return skills.get_current();
}

[[nodiscard]] Status Tactic::getStatus() const {
    return currentStatus;
}
}  // namespace rtt::ai::stp
