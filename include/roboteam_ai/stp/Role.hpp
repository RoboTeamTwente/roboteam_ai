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
    [[nodiscard]] virtual Status update(StpInfo const& info) noexcept;

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
    collections::state_machine<Tactic, Status, StpInfo> robotTactics;

    /**
     * This function should calculate any extra information that the tactics might need to be executed.

     * Though this method is responsible for ensuring everything is calculated, it helps to use helpers so this
     * function doesn't become a massive hack
     */
    virtual StpInfo calculateInfoForTactic(StpInfo const &info) noexcept = 0;

    /**
     * When the state should reset
     * @param info the role info passed down from the play
     * @return true if the active tactic cannot execute (it's prerequisites are no longer met)
     */
    virtual bool shouldRoleReset(const StpInfo &info) noexcept = 0;
};

}  // namespace rtt::ai::stp

#endif  // RTT_ROLE_HPP
