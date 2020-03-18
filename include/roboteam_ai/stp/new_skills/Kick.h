//
// Created by jordi on 03-03-20.
//

#ifndef RTT_KICK_H
#define RTT_KICK_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp::skill {

class Kick : public Skill {
    /**
     * On initialize of this tactic
     */
    void onInitialize() noexcept override;

    /**
     * On update of this tactic
     */
    Status onUpdate(StpInfo const& info) noexcept override;

    /**
     * On terminate of this tactic
     */
    void onTerminate() noexcept override;
};

}  // namespace rtt::ai::stp::skill

#endif  // RTT_KICK_H
