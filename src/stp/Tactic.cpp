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

    // Update skill info
    auto skill_info = calculateInfoForSkill(info);

    // Update the current skill with the new SkillInfo
    auto status = skills.update(skill_info);

    // Call onUpdate on a skill for specific behaviour
    onUpdate(status);
    RTT_DEBUG("ID AFTER UPDATE: ", skills.current_num(), " Called on robot: ", info.getRobot()->get()->getId());

    // Check if the skills are all finished
    if (skills.finished()) {
        RTT_INFO("TACTIC SUCCESSFUL!!!!!!!!!!!!!!!!!!!!!!!!:)")
        if (!isEndTactic()) {
            return Status::Success;
        }
        skills.skip_n(-1);
        return Status::Waiting;
    }

    // if the failing condition is true, the current tactic will fail
    if(isTacticFailing(info)){
        RTT_INFO("Current Tactic Failed for ID = ", info.getRobot()->get()->getId())
        return Status::Failure;
    }

    // the tactic will not be reset if it's the first skill
    if((skills.current_num() != 0 && shouldTacticReset(info))){
        RTT_INFO("State Machine reset for current tactic for ID = ", info.getRobot()->get()->getId())
        // TODO: messy reset, do it in the state machine
        skills.skip_n(- skills.current_num());
    }

    return Status::Running;
}

void Tactic::terminate() noexcept { onTerminate(); }

}  // namespace rtt::ai::stp
