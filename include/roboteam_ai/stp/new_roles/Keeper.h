//
// Created by jordi on 08-04-20.
//

#ifndef RTT_KEEPER_H
#define RTT_KEEPER_H

#include "stp/Role.hpp"
#include "world/Field.h"

namespace rtt::ai::stp::role {

class Keeper : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    explicit Keeper(std::string name);

    /**
     * Besides the default update from base class Role, it also switches between tactics depending on the ball position and velocity
     * @param info TacticInfo to be passed to update()
     * @return The status that the current tactic returns
     */
    [[nodiscard]] Status update(StpInfo const& info) noexcept override;

   private:
    /**
     * Checks if ball is in our defense area and still
     * @param field Field
     * @param ballPos Ball position
     * @param ballVel Ball velocity
     * @return True if ball is in our defense area and still
     */
    [[nodiscard]] static bool isBallInOurDefenseAreaAndStill(const world::Field& field, const Vector2& ballPos, const Vector2& ballVel) noexcept;

    /**
     * Resets state machine when ball is in our defense area and still and current tactic is not KeeperBlockBall
     * @param isBallInOurDefenseAreaAndStill True if ball is in our defense area and still
     * @return True if isBallInOurDefenseAreaAndStill and current tactic is not KeeperBlockBall
     */
    [[nodiscard]] bool shouldRoleReset(bool isBallInOurDefenseAreaAndStill) noexcept;
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_KEEPER_H
