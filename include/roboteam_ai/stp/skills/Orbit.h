//
// Created by mamiksik on 22-04-21.
//

#ifndef RTT_ORBIT_H
#define RTT_ORBIT_H

#include "roboteam_utils/pid.h"
#include "stp/Skill.h"

namespace rtt::ai::stp::skill {

class Orbit : public Skill {
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

    /**
     * PID controller to determine velocity multiplier
     */
    PID velPid = PID(0.75, 0, 0);
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_ORBIT_H
