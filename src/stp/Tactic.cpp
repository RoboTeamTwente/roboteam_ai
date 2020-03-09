//
// Created by jessevw on 03.03.20.
//

#include "include/roboteam_ai/stp/Tactic.h"
#include "stp/StpInfo.h"

namespace rtt::ai::stp {

TacticInfo Tactic::calculateInfoForSkill() {
}

Status Tactic::initialize() noexcept {
    return onInitialize();
}
Status Tactic::update(TacticInfo const &info) noexcept {
    return onUpdate(calculateInfoForSkill());
}
Status Tactic::terminate() noexcept {
    return onTerminate();
}
}  // namespace rtt::ai::stp
