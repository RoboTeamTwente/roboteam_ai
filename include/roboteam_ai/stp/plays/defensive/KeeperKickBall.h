//
// Created by Alexander on 29/01/2022.
//

#ifndef RTT_KEEPERKICKBALL_H
#define RTT_KEEPERKICKBALL_H

#include "stp/Play.hpp"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {

/**
 * KeeperKickBall Play is executed when the ball is in our defense area and should be kicked out by our keeper
 */

class KeeperKickBall : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    KeeperKickBall();

    /**
     * Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::world::Field& field) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * Calculates info for the midfielders
     */
    void calculateInfoForMidfielders() noexcept;

    /**
     * Calculates info for the attackers
     */
    void calculateInfoForAttackers() noexcept;

    /**
     * Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World* world) noexcept override{};

    /**
     * Gets the play name
     */
    const char* getName() override;

    /**
     * Check if play should end. True if pass arrived, if the ball is not moving anymore after pass, or if there is a better pass available
     */
    bool shouldEndPlay() noexcept override;

   private:
    /**
     * Return true if passer is done with KickAtPos tactic
     */
    bool ballKicked();

    /**
     * Struct containing info about the pass. Calculated once for each time this play is run
     */
    PassInfo passInfo;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_KEEPERKICKBALL_H
