//
// Created by jessevw on 03.03.20.
//

#include "stp/Tactic.h"

#include <roboteam_utils/Print.h>

#include "stp/StpInfo.h"

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

    // Update the current skill with the new SkillInfo
    auto status = skills.update(skill_info);

    // Call onUpdate on a skill for specific behaviour
    onUpdate(status);
    RTT_DEBUG("ID AFTER UPDATE: ", skills.current_num(), " Called on robot: ", info.getRobot()->get()->getId())

    // Check if the skills are all finished
    if (skills.finished()) {
        RTT_INFO("TACTIC SUCCESSFUL for ", info.getRobot()->get()->getId())
        if (!isEndTactic()) {
            currentStatus = Status::Success;
            return Status::Success;
        }
        // Make sure we keep executing the last tactic since it is an end tactic
        skills.skip_n(-1);
        currentStatus = Status::Waiting;
        return Status::Waiting;
    }

    // if the failing condition is true, the current tactic will fail
    if (isTacticFailing(skill_info)) {
        RTT_INFO("Current Tactic Failed for ID = ", info.getRobot()->get()->getId())
        currentStatus = Status::Failure;
        return Status::Failure;
    }

    // the tactic will not be reset if it's the first skill
    if ((skills.current_num() != 0 && shouldTacticReset(skill_info))) {
        RTT_INFO("State Machine reset for current tactic for ID = ", info.getRobot()->get()->getId())
        reset();
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

    const char *Tactic::getName() {
        return "[abc] Tactic";
    }
}  // namespace rtt::ai::stp
