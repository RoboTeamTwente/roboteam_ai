//
// Created by jordi on 11-03-20.
//

#ifndef RTT_HASBALL_H
#define RTT_HASBALL_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp {

    class HasBall : public Skill {
        void onInitialize() noexcept override;
        Status onUpdate(SkillInfo const& info) noexcept override;
        void onTerminate() noexcept override;
    };

} // namespace rtt::ai::stp


#endif //RTT_HASBALL_H
