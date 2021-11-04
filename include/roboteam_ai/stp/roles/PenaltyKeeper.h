//
// Created by jordi on 08-04-20.
// Modified by timovdk on 4/29/20.
//

#ifndef RTT_PENALTYKEEPER_H
#define RTT_PENALTYKEEPER_H

#include "Keeper.h"
#include "stp/Role.hpp"
#include "world/Field.h"

namespace rtt::ai::stp::role {

//
// Role inherited from the Keeper
//

class PenaltyKeeper : public Keeper {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    explicit PenaltyKeeper(std::string name);

    /**
     * Besides the default update from base class Role, it also switches between tactics depending on the ball position and velocity
     * @param info TacticInfo to be passed to update()
     * @return The status that the current tactic returns
     */
    [[nodiscard]] Status update(StpInfo const& info) noexcept override;
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_PENALTYKEEPER_H
