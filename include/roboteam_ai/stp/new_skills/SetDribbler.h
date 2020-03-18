//
// Created by timovdk on 2/11/20.
//

#ifndef RTT_SETDRIBBLER_H
#define RTT_SETDRIBBLER_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp::skill {

class SetDribbler : public Skill {
   public:
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

#endif  // RTT_SETDRIBBLER_H
