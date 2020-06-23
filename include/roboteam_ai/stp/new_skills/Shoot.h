//
// Created by rtt-vision on 23-06-20.
//

#ifndef RTT_SHOOT_H
#define RTT_SHOOT_H


#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp::skill {

    class Shoot : public Skill {
        /**
         * On initialize of this tactic
         */
        void onInitialize() noexcept override;

        /**
         * On update of this tactic
         */
        Status onUpdate(StpInfo const& info) noexcept override;

        Status onUpdateChip(StpInfo const& info) noexcept;

        Status onUpdateKick(StpInfo const& info) noexcept;

        /**
         * On terminate of this tactic
         */
        void onTerminate() noexcept override;

        /**
         * Gets the skill name
         */
        const char *getName() override;
    };

}  // namespace rtt::ai::stp::skill

#endif //RTT_SHOOT_H
