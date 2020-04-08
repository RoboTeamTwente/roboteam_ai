//
// Created by timovdk on 3/13/20.
//

#ifndef RTT_CHIPATPOS_H
#define RTT_CHIPATPOS_H

#include "include/roboteam_ai/stp/Tactic.h"

namespace rtt::ai::stp::tactic {

class ChipAtPos : public Tactic {
   public:
    ChipAtPos();

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
     * Calculate the SkillInfo from the TacticInfo
     * @param info info is the TacticInfo passed by the role
     * @return SkillInfo based on the TacticInfo
     */
    StpInfo calculateInfoForSkill(StpInfo const &info) noexcept override;

    /**
     * Calculate the chipforce for the skill
     * @param distance distance to the target
     * @param desiredBallSpeedType type of the chip
     * @return the speed the chipper needs to chip at
     */
    double determineChipForce(double distance, KickChipType desiredBallSpeedType) noexcept;

    bool isEndTactic() noexcept override;

    bool isTacticFailing(const StpInfo &info) noexcept override;

    bool shouldTacticReset(const StpInfo &info) noexcept override;

    /**
     * Gets the tactic name
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_CHIPATPOS_H
