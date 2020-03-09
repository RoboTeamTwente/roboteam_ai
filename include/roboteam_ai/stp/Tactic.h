//
// Created by jessevw on 03.03.20.
//

#ifndef RTT_TACTIC_H
#define RTT_TACTIC_H

#include <vector>
#include <roboteam_utils/containers/state_machine.hpp>

#include "stp/StpInfo.h"
#include "stp/new_skills/Rotate.h"
#include "stp/new_skills/SetDribbler.h"
namespace rtt::ai::stp {
class Skill;

class Tactic {
   private:
    rtt::collections::state_machine<Skill, Status, SkillInfo> skills{Rotate(), SetDribbler(), Rotate(), Rotate()};

    /**
     * This function should calculate any extra information that the skills might need to be executed.
     * Things that should be calculated are for example how hard the kicker should shoot to get to the desired position
     * or how fast the dribbler should be spinning.
     *
     * Though this method is responsible for ensuring everything is calculated, it helps to use helpers so this
     * function doesn't become a massive hack
     */
    TacticInfo calculateInfoForSkill();

    virtual Status onInitialize() noexcept = 0;
    virtual Status onUpdate(TacticInfo const& info) noexcept = 0;
    virtual Status onTerminate() noexcept = 0;

    public:

    virtual Status initialize() noexcept;
    virtual Status update(TacticInfo const& info) noexcept;
    virtual Status terminate() noexcept;
};
}  // namespace rtt::ai::stp

#endif  // RTT_TACTIC_H
