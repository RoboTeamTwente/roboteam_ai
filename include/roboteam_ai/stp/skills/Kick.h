//
// Created by jordi on 03-03-20.
//

#ifndef RTT_KICK_H
#define RTT_KICK_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {

class Kick : public Skill {
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
};
}  // namespace rtt::ai::stp::skill

#endif  // RTT_KICK_H
