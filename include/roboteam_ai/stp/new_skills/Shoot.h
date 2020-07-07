//
// Created by Jesse on 23-06-20.
//

#ifndef RTT_SHOOT_H
#define RTT_SHOOT_H

#include "stp/Skill.h"

namespace rtt::ai::stp::skill {

class Shoot : public Skill {
    /**
     * On initialize of this tactic
     */
    void onInitialize() noexcept override;

    /**
     * On update of this tactic
     * Here we check if we should kick or chip and call the corresponding function
     */
    Status onUpdate(StpInfo const& info) noexcept override;

    /**
     * This function handles chipping
     * @param info
     * @return Status
     */
    Status onUpdateChip(StpInfo const& info) noexcept;

    /**
     * This function handles kicking
     * @param info
     * @return Status
     */
    Status onUpdateKick(StpInfo const& info) noexcept;

    /**
     * On terminate of this tactic
     */
    void onTerminate() noexcept override;

    /**
     * Gets the skill name
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
