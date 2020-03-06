//
// Created by jordi on 03-03-20.
//

#ifndef RTT_KICK_H
#define RTT_KICK_H

#include <include/roboteam_ai/stp/Skill.h>

namespace rtt::ai::stp {

class Kick : public Skill {
    Status onInitialize() noexcept override;
    Status onUpdate(SkillInfo const& info) noexcept override;
    Status onTerminate() noexcept override;
};

} // namespace rtt::ai

#endif //RTT_KICK_H
