//
// Created by jessevw on 03.03.20.
//

#ifndef RTT_TACTIC_H
#define RTT_TACTIC_H

#include <roboteam_utils/containers/state_machine.hpp>
#include <vector>

#include "stp/Skill.h"
#include "stp/StpInfo.h"

namespace rtt::ai::stp {
class Skill;

class Tactic {
   protected:
    /**
     * This function should calculate any extra information that the skills might need to be executed.
     * Things that should be calculated are for example how hard the kicker should shoot to get to the desired position
     * or how fast the dribbler should be spinning.
     *
     * Though this method is responsible for ensuring everything is calculated, it helps to use helpers so this
     * function doesn't become a massive hack
     */
    virtual StpInfo calculateInfoForSkill(StpInfo const &info) noexcept = 0;

    /**
     * called on initialization of this tactic
     */
    virtual void onInitialize() noexcept = 0;

    /**
     * called on update of this tactic
     */
    virtual void onUpdate(Status const &status) noexcept = 0;

    /**
     * called on terminate of this tactic
     */
    virtual void onTerminate() noexcept = 0;

    /**
     * The state machine of skills
     */
    rtt::collections::state_machine<Skill, Status, StpInfo> skills;

   public:
    /**
     * Calls onInitialize of the tactic
     */
    virtual void initialize() noexcept;

    /**
     * Check if state machine is done, calls calculateInfoForSkill, calls update on the state machine with SkillInfo and calls onUpdate of this tactic for extra customization
     * @param info info passed by the Role
     * @return Status of the skill that is currently being ticked
     */
    virtual Status update(StpInfo const &info) noexcept;

    /**
     * Calls onTerminate
     */
    virtual void terminate() noexcept;
};
}  // namespace rtt::ai::stp

#endif  // RTT_TACTIC_H
