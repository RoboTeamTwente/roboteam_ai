//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Role.hpp"

namespace rtt::ai::stp {
Status Role::update(StpInfo const& info) noexcept {
    // Check if the skills are all finished
    if (robotTactics.finished()) {
        return Status::Success;
    }

    // Update the state machine of tactics with the TacticInfo from Play
    return robotTactics.update(info);
}

bool Role::finished() const noexcept { return robotTactics.finished(); }

}  // namespace rtt::ai::stp