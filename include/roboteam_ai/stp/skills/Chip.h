//
// Created by jordi on 09-03-20.
//
//Skill for chipping

#ifndef RTT_CHIP_H
#define RTT_CHIP_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {

class Chip : public Skill {
    /**
     * On update of this tactic
     * @param info StpInfo struct with all relevant info for this robot and this skill
     * @return A Status, either Running or Success
     */
    Status onUpdate(StpInfo const& info) noexcept override;

    /**
     * Gets the skill name
     * @return The name of this skill
     */
    const char* getName() override;

   private:
    /**
     * Keeps track of how many ticks we tried to chip
     */
    int chipAttempts = 0;
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_CHIP_H
