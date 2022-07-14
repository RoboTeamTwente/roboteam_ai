//
// Created by jessevw on 03.03.20.
//

#include "stp/Tactic.h"

#include <roboteam_utils/Print.h>

namespace rtt::ai::stp {

void Tactic::initialize() noexcept {
    for (auto &x : skills) {
        x->initialize();
    }
}

Status Tactic::update(StpInfo const &info) noexcept {
    // Update skill info
    auto skill_info = calculateInfoForSkill(info);

    if (skill_info) {
        // Update the current skill with the new SkillInfo
        auto status = skills.update(skill_info.value());

        (void)status;  // return of update is never used, but it is marked [[no-discard]] in the state machine. This cast suppresses that warning.
    } else {
        RTT_ERROR(getName(), "Not all data was present, bad update!")
        return Status::Failure;
    }

    // the tactic will not be reset if it's the first skill
    if (skills.current_num() != 0 && shouldTacticReset(skill_info.value())) {
        reset();
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

    // if the failing condition is true, the current tactic will fail
    if (skill_info && isTacticFailing(skill_info.value())) {
        currentStatus = Status::Failure;
        return Status::Failure;
    }

    currentStatus = Status::Running;
    return Status::Running;
}

void Tactic::terminate() noexcept {
    for (auto &x : skills) {
        x->terminate();
    }
}

void Tactic::reset() noexcept { skills.reset(); }

Skill *Tactic::getCurrentSkill() { return skills.get_current(); }

[[nodiscard]] Status Tactic::getStatus() const { return currentStatus; }

bool Tactic::forceTacticSuccess(const StpInfo &info) noexcept { return false; }
}  // namespace rtt::ai::stp
