//
// Created by doormat on 17-11-22.
//

#include "stp/roles/active/ChippingPass.h"

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/ChipAtPos.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role{

ChippingPass::ChippingPass(std::string name) : Role(std::move(name)) {
    //create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic,Status,StpInfo>{tactic::GetBall(),tactic::ChipAtPos(),tactic::Formation()};
}
}// namespace rtt::ai::stp::role