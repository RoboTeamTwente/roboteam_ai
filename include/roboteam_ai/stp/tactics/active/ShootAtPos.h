//
// Created by Jesse on 23-06-20.
//

#ifndef RTT_SHOOTATPOS_H
#define RTT_SHOOTATPOS_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {

class ShootAtPos : public Tactic {
   public:
    /**
     * Constructor for the tactic, it constructs the state machine of skills
     */
    ShootAtPos();

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
     * Fails if robot doesn't have the ball or if there is no shootTarget
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * Should this tactic be reset (go back to the first skill of this tactic)
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     * Reset if ball is moving or if the angle of the robot to the ball is greater than some error margin
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
     * Calculate the info if the role/play wants to execute a kick
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForKick(const StpInfo &info) noexcept;

    /**
     * Calculate the info if the role/play wants to execute a chip
     * @param info info is the StpInfo passed by the role
     * @return std::optional<SkillInfo> based on the StpInfo parameter
     */
    std::optional<StpInfo> calculateInfoForChip(const StpInfo &info) noexcept;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_SHOOTATPOS_H
