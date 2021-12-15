//
// Created by jordi on 06-03-20.
//

#ifndef RTT_ROTATE_H
#define RTT_ROTATE_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {

class Rotate : public Skill {
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

    /**
     * Counts how long the robot is within the rotational error margin
     */
    int withinMarginCount = 0;
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_ROTATE_H
