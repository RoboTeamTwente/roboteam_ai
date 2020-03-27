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
    Role(std::string name)
        : roleName{std::move(name)} {}

    /**
    * Function that's called every tick, default implementation is robotTactics.update();
    * @param info TacticInfo to be passed to update()
    * @return The status that the current tactic returns
    */
    [[nodiscard]] virtual Status update(StpInfo const &info) noexcept;

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
    [[nodiscard]] std::optional<world_new::view::RobotView> const& getCurrentRobot() const;

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
     * Forces the Role to skip to tactic n in the state machine
     * @param n Index of the tactic in the state machine
     */
    void forceToTactic(int n) noexcept;

protected:
    /**
     * Robot to which this role is currently assigned
     */
    std::optional<world_new::view::RobotView> currentRobot;

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
