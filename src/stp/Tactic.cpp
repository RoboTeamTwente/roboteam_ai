//
// Created by jessevw on 03.03.20.
//

#include "include/roboteam_ai/stp/Tactic.h"
#include <roboteam_utils/Print.h>
#include "stp/StpInfo.h"

namespace rtt::ai::stp {

void Tactic::initialize() noexcept { onInitialize(); }

Status Tactic::update(StpInfo const &info) noexcept {
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info");
        return Status::Failure;
    }

    // Check if the skills are all finished
    if (skills.finished() && !isEndTactic()) {
        // Tactic tests don't have robotID, so make sure it does not segfault here
        if (info.getRobot().value()) RTT_INFO("TACTIC SUCCESSFUL for ", info.getRobot()->get()->getId())
        return Status::Success;
    }

    // Update skill info
    auto skill_info = calculateInfoForSkill(info);

    // if the failing condition is true, the current tactic will fail
    if(isTacticFailing(skill_info)){
        RTT_INFO("Current Tactic Failed for ID = ", info.getRobot()->get()->getId())
        return Status::Failure;
    }

    // the tactic will not be reset if it's the first skill; it will be reset as well if the
    // state machine is finished
    if(skills.finished() || (skills.current_num() != 0 && shouldTacticReset(skill_info))){
        RTT_INFO("State Machine reset for current tactic for ID = ", info.getRobot()->get()->getId())
        skills.reset();
    }

    // Update the current skill with the new SkillInfo
    auto status = skills.update(skill_info);

    // Call onUpdate on a skill for specific behaviour
    onUpdate(status);

    return Status::Running;
}

void Tactic::terminate() noexcept { onTerminate(); }

}  // namespace rtt::ai::stp
