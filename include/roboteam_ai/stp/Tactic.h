//
// Created by jessevw on 03.03.20.
//

#ifndef RTT_TACTIC_H
#define RTT_TACTIC_H

#include <vector>
#include <roboteam_utils/containers/state_machine.hpp>
#include "stp/StpInfo.h"
#include "stp/Skill.h"

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
        virtual SkillInfo calculateInfoForSkill(TacticInfo const &info) noexcept = 0;

        virtual void onInitialize() noexcept = 0;
        virtual Status onUpdate(SkillInfo const &info) noexcept = 0;
        virtual void onTerminate() noexcept = 0;

    protected:
        rtt::collections::state_machine<Skill, Status, SkillInfo> skills;

    public:
        virtual void initialize() noexcept;
        virtual Status update(TacticInfo const &info) noexcept;
        virtual void terminate() noexcept;
};
}  // namespace rtt::ai::stp

#endif  // RTT_TACTIC_H
