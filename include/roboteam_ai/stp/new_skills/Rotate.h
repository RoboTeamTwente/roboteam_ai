//
// Created by jordi on 06-03-20.
//

#ifndef RTT_ROTATE_H
#define RTT_ROTATE_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp {

class Rotate : public Skill {
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

#endif  // RTT_ROTATE_H
