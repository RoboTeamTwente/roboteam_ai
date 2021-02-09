//
// Created by timo on 3/30/20.
//

#ifndef RTT_AGGRESSIVESTOPFORMATION_H
#define RTT_AGGRESSIVESTOPFORMATION_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class AggressiveStopFormation : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    AggressiveStopFormation();

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(world::World* world) noexcept override;

    /**
     * Decides the input for the robot dealer. The result will be used to distribute the roles
     * @return a mapping between roles and robot flags, used by the robot dealer to assign roles
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Gets the play name
     */
    const char* getName() override;

   protected:
    /**
     * This function is used to determine if -- when a role is in an endTactic -- the endTactic should be skipped.
     * An example could be BlockRobot and Intercept. You block a robot (endTactic) until a ball is shot and then the robot
     * closest to the ball should try to intercept (skip the BlockRobot tactic to execute Intercept)
     */
    bool shouldRoleSkipEndTactic() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_AGGRESSIVESTOPFORMATION_H
