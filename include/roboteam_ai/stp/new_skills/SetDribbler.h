//
// Created by timovdk on 2/11/20.
//

#ifndef RTT_SETDRIBBLER_H
#define RTT_SETDRIBBLER_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp {

class SetDribbler : public Skill {
   public:
    /**
     * On initialize of this tactic
     */
    void onInitialize() noexcept override;

    /**
     * On update of this tactic
     */
    Status onUpdate(SkillInfo const& info) noexcept override;

    /**
     * On terminate of this tactic
     */
    void onTerminate() noexcept override;
};

}  // namespace rtt::ai::stp

#endif  // RTT_SETDRIBBLER_H
