//
// Created by jordi on 09-03-20.
//

#ifndef RTT_CHIP_H
#define RTT_CHIP_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp::skill {

class Chip : public Skill {
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

    /**
     * Gets the skill name
     */
    const char* getName() override;

    private:
        int chipAttempts = 0;
};

}  // namespace rtt::ai::stp::skill

#endif  // RTT_CHIP_H
