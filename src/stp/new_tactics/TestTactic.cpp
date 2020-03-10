//
// Created by roboteam on 9/3/20.
//

#include "include/roboteam_ai/stp/new_tactics/TestTactic.h"

namespace rtt::ai::stp {

Status TestTactic::onInitialize() noexcept {
    return Status::Success;
}

Status TestTactic::onUpdate(const rtt::ai::stp::TacticInfo &info) noexcept {
    return Status::Success;
}

Status TestTactic::onTerminate() noexcept {
    return Status::Success;
}

} // namespace rtt::ai::stp

