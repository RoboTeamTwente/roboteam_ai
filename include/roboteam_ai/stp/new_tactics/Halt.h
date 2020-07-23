//
// Created by jessevw on 24.03.20.
//

#ifndef RTT_HALT_TACTIC_H
#define RTT_HALT_TACTIC_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * This tactic is for halting, it rotates the robot to angle 0
 * It cannot fail and reset. It's not an
 * end tactic, therefore it can succeed.
 */
class Halt : public Tactic {
   public:
    /**
     * Constructor for the tactic, it constructs the state machine of skills
     */
    Halt();

   private:
    /**
     * Calculate the info for skills from the StpInfo struct parameter
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * Is this tactic failing during execution (go back to the previous tactic)
     * @param info StpInfo can be used to check some data
     * @return true, tactic will fail (go back to prev tactic), false execution will continue as usual
     * This tactic can never fail, so always returns false
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * Should this tactic be reset (go back to the first skill of this tactic)
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     * This tactic can never reset, so always returns false
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * Is this tactic an end tactic?
     * @return This will always return true, since it is an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_HALT_TACTIC_H
