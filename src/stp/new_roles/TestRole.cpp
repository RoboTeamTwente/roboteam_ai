//
// Created by timovdk on 3/10/20.
//

#include <roboteam_utils/Print.h>
#include <stp/new_roles/TestRole.h>
#include <stp/new_tactics/KickAtPos.h>
#include <stp/new_tactics/Intercept.h>

#include <utility>
#include <include/roboteam_ai/stp/new_tactics/GetBall.h>

namespace rtt::ai::stp {

TestRole::TestRole(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall()};
    robotTactics.initialize();
}

StpInfo TestRole::calculateInfoForTactic(StpInfo const &info) noexcept {
    StpInfo tacticInfo = info;
    return tacticInfo;
};


bool TestRole::shouldRoleReset(const StpInfo &info) noexcept {
    return false;
};

}  // namespace rtt::ai::stp