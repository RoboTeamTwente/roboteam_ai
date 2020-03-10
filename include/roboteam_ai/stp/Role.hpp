//
// Created by john on 3/9/20.
//

#ifndef RTT_ROLE_HPP
#define RTT_ROLE_HPP
#include <utility>
#include <vector>

#include "Tactic.h"

namespace rtt::ai::stp {
/**
 * Role class used in STP, pretty self explanatory
 * essentially a state machine of Tactics
 */
class Role {
   public:
    Role(std::string name) : roleName{std::move(name)} {}
    /**
     * Function that's called every tick, default implementation is robotTactics.update();
     * @param info TacticInfo to be passed to update()
     * @return The status that the current tactic returns
     */
    [[nodiscard]] virtual Status update(stp::TacticInfo const& info) noexcept;

    /**
     * @return True if all tactics returned Status::finish
     */
    [[nodiscard]] bool finished() const noexcept;

    /**
     * Gets the name
     * @return name of the role
     */
    std::string getName() { return roleName; }

   protected:
    /**
     * Name of the role
     */
    std::string roleName{};

    /**
     * State machine that keeps track of tactic states
     */
    collections::state_machine<Tactic, Status, TacticInfo> robotTactics;
};

}  // namespace rtt::ai::stp

#endif  // RTT_ROLE_HPP
