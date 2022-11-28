//
// Created by tijmen on 01-07-22.
//

#ifndef RTT_ORBITANGULAR_H
#define RTT_ORBITANGULAR_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {

class OrbitAngular : public Skill {
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
     * Counter for how many ticks the robot is within the error margin
     */
    int counter = 0;
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_ORBITANGULAR_H
