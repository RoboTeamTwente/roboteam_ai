//
// Created by john on 3/9/20.
//

#ifndef RTT_ROLE_HPP
#define RTT_ROLE_HPP

#include <vector>

#include "Tactic.h"

namespace rtt::ai::stp {
/**
 * Role class used in STP
 * Every robot needs to have a role
 * A role is essentially a state machine of Tactics
 */
class Role {
   public:
    /**
     * Constructor that has a name as its parameter
     * @param name the name of the role
     */
    explicit Role(std::string name) : roleName{std::move(name)} {}

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

    /**
     * Gets the current robot
     * @return view to the robot this role belongs to, optional.
     */
    [[nodiscard]] std::optional<world::view::RobotView> const& getCurrentRobot() const;

    /**
     * Gets the tactic whose turn it is
     * @return Tactic*
     */
    [[nodiscard]] Tactic* getCurrentTactic();

    /**
     * Forces the Role to skip to the next tactic in the state machine
     */
    void forceNextTactic() noexcept;

    /**
     * Resets the tactics, skills and robot of this role so re-dealing of robots works as expected
     */
    void reset() noexcept;

    /**
     * Virtual default dtor, ensures proper destruction of Role
     */
    virtual ~Role() = default;

    /**
     * Default cpy ctor, ensures proper copy construction of Role
     */
    Role(Role& other) = default;

    /**
     * Default mv ctor, ensures proper move construction of Role
     */
    Role(Role&& other) = default;

   protected:
    /**
     * Robot to which this role is currently assigned
     */
    std::optional<world::view::RobotView> currentRobot;

    /**
     * Name of the role
     */
    std::string roleName{};

    /**
     * State machine that keeps track of tactic states
     */
    collections::state_machine<Tactic, Status, StpInfo> robotTactics;
};
}  // namespace rtt::ai::stp

#endif  // RTT_ROLE_HPP
