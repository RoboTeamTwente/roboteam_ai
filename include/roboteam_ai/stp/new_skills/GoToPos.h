//
// Created by jordi on 09-03-20.
//

#ifndef RTT_GOTOPOS_H
#define RTT_GOTOPOS_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp {

class GoToPos : public Skill {
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

#endif  // RTT_GOTOPOS_H
