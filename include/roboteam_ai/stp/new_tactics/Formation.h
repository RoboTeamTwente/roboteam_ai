//
// Created by timovdk on 3/27/20.
//

#ifndef RTT_FORMATION_TACTIC_H
#define RTT_FORMATION_TACTIC_H

#include "stp/Tactic.h"

namespace rtt::ai::stp::tactic {

class Formation : public Tactic {
   public:
    Formation();

   private:
    /**
     * On initialization of this tactic, initialize the state machine with skills
     */
    void onInitialize() noexcept override;

    /**
     * On update of this tactic
     */
    void onUpdate(Status const &status) noexcept override;

    /**
     * On terminate of this tactic, call terminate on all underlying skills
     */
    void onTerminate() noexcept override;

    /**
     * Calculate info for the skills
     * @param info Info passed by the role
     * @return std::optional<SkillInfo> based on the TacticInfo
     */
    std::optional<StpInfo> calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * Tactic fails if target type is not a move target
     * @param info Info
     * @return True if target type is not a move target
     */
    bool isTacticFailing(const StpInfo &info) noexcept override;

    /**
     * Reset tactic when robot position is not close enough to the target position
     * @param info Info
     * @return True if robot position is not close enough to the target position
     */
    bool shouldTacticReset(const StpInfo &info) noexcept override;

    bool isEndTactic() noexcept override;

    /**
     * Gets the tactic name
     */
    const char *getName() override;
};

}  // namespace rtt::ai::stp::tactic

#endif  // RTT_FORMATION_TACTIC_H
