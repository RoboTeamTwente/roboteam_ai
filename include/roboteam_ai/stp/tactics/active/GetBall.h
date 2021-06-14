//
// Created by ratoone on 10-03-20.
//

#ifndef RTT_GETBALL_H
#define RTT_GETBALL_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {
/**
 * This tactic is for getting the ball. It has 3 skills: GoToPos, Rotate, and SetDribbler.
 * It cannot fail, and it's getting reset when there the robot loses the ball. It's not an
 * end tactic, therefore it can succeed.
 */
class GetBall : public Tactic {
   public:
    /**
     * Constructor for the tactic, it constructs the state machine of skills
     */
    GetBall();

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
     * Returns true when the robot does not have the ball
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * Is this tactic an end tactic?
     * @return This will always return false, since it is NOT an endTactic
     */
    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     * @return The name of this tactic
     */
    const char *getName() override;

    /**
     * Checks if this tactic should be forced success
     * It is forced success whenever the corresponding robot has the ball
     * @param info
     * @return whether the tactic is forced success
     */
    bool forceTacticSuccess(const StpInfo &info) noexcept override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_GETBALL_H
