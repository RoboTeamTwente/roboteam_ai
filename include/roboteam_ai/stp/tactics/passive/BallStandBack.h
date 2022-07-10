//
// Created by agata on 29/06/2022.
//

#ifndef RTT_BALLSTANDBACK_H
#define RTT_BALLSTANDBACK_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {

class BallStandBack : public Tactic {
   public:
    /**
     * Constructor for the tactic, it constructs the state machine of skills
     */
    BallStandBack();

   private:
    // We want to stand still for a bit to make sure the ball doesnt have backspin
    int standStillCounter = 0;

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
     * Fails if there is no position to move to
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * Should this tactic be reset (go back to the first skill of this tactic)
     * @param info StpInfo can be used to check some data
     * @return true if tactic  should reset, false if execution should continue
     * Resets when robot position is not close enough to the target position
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

#endif  // RTT_BALLSTANDBACK_H
