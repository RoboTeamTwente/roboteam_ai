//
// Created by jessevw on 03.03.20.
//

#include "include/roboteam_ai/stp/Tactic.h"
#include "stp/StpInfo.h"

namespace rtt::ai::stp {

void Tactic::initialize() noexcept {
    onInitialize();
}
Status Tactic::update(TacticInfo const &info) noexcept {
    return onUpdate(calculateInfoForSkill(info));
}
void Tactic::terminate() noexcept {
    onTerminate();
}
}  // namespace rtt::ai::stp
