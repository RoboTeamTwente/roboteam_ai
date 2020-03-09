//
// Created by jordi on 09-03-20.
//

#ifndef RTT_CHIP_H
#define RTT_CHIP_H

#include "include/roboteam_ai/stp/Skill.h"

namespace rtt::ai::stp {

class Chip : public Skill {
    void onInitialize() noexcept override;
    Status onUpdate(SkillInfo const& info) noexcept override;
    void onTerminate() noexcept override;
};

} // namespace rtt::ai::stp

#endif //RTT_CHIP_H
