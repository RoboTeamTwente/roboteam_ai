//
// Created by jessevw on 03.03.20.
//

#ifndef RTT_TACTIC_H
#define RTT_TACTIC_H

#include <roboteam_utils/containers/state_machine.hpp>
#include <vector>
#include <roboteam_utils/Print.h>
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
     * The condition when the current tactic fails
     * @param info the tactic info passed down from the play
     * @return true if the tactic's prerequisites are no longer met (e.g. losing the ball)
     */
    virtual bool isTacticFailing(const StpInfo &info) noexcept = 0;

    /**
     * When the state should reset
     * @param info the tactic info passed down from the play
     * @return true if the active skill cannot execute (it's prerequisites are no longer met)
     */
    virtual bool shouldTacticReset(const StpInfo &info) noexcept = 0;

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

    /**
     * Check if the current tactic is an end tactic - only Running or Failure status
     * @return true if the current tactic cannot succeed (i.e. is an end tactic); default false
     */
    virtual bool isEndTactic() noexcept;
};
}  // namespace rtt::ai::stp

#endif  // RTT_TACTIC_H
