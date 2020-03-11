//
// Created by jordi on 11-03-20.
//

#include "include/roboteam_ai/stp/new_skills/HasBall.h"

namespace rtt::ai::stp {

    void HasBall::onInitialize() noexcept { }

    Status HasBall::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
        if (info.getRobot()->hasBall()) {
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }

    void HasBall::onTerminate() noexcept { }

} // namespace rtt::ai::stp