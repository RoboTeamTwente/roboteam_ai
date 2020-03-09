//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Role.hpp"

namespace rtt::ai::stp {
    Status Role::update(const stp::TacticInfo &info) noexcept {
        return robotTactics.update(info);
    }

    bool Role::finished() const noexcept {
        return robotTactics.finished();
    }

} // namespace rtt::ai::stp