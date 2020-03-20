//
// Created by timovdk on 3/10/20.
//

#include <roboteam_utils/Print.h>
#include <stp/new_roles/TestRole.h>
#include <stp/new_tactics/KickAtPos.h>
#include <stp/new_tactics/Intercept.h>

#include <utility>

namespace rtt::ai::stp {

TestRole::TestRole(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    if (getName() == "kicker") {
        std::cerr << "Kicker\n";
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::KickAtPos()};
    } else{
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Intercept()};
}

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