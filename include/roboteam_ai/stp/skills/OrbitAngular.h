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
     * Current direction the robot is turning in. 1 for clockwise, -1 for counterclockwise
     */
    double currentDirection = 1;
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_ORBITANGULAR_H
