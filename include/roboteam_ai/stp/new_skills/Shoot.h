//
// Created by Jesse on 23-06-20.
//

#ifndef RTT_SHOOT_H
#define RTT_SHOOT_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {

class Shoot : public Skill {
    /**
     * On update of this tactic
     * Here we check if we should kick or chip and call the corresponding function
     * @param info StpInfo struct with all relevant info for this robot and this skill
     * @return A Status, either Running or Success
     */
    Status onUpdate(StpInfo const& info) noexcept override;

    /**
     * This function handles chipping
     * @param info StpInfo struct with all relevant info for this robot and this skill
     * @return A Status, either Running or Success
     */
    Status onUpdateChip(StpInfo const& info) noexcept;

    /**
     * This function handles kicking
     * @param info StpInfo struct with all relevant info for this robot and this skill
     * @return A Status, either Running or Success
     */
    Status onUpdateKick(StpInfo const& info) noexcept;

    /**
     * Gets the skill name
     * @return The name of this skill
     */
    const char* getName() override;

   private:
    /**
     * Keeps track of how many ticks we tried to shoot
     */
    int shootAttempts = 0;
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_SHOOT_H
