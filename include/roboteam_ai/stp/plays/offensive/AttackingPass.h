//
// Created by jessevw on 17.03.20.
//

#ifndef RTT_ATTACKING_PASS_PLAY_H
#define RTT_ATTACKING_PASS_PLAY_H

#include <roboteam_utils/Grid.h>

#include "stp/Play.hpp"
#include "stp/computations/PassComputations.h"

namespace rtt::ai::stp::play {

class AttackingPass : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    AttackingPass();

    /**
     *  Calculate how beneficial we expect this play to be
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
     * Calculates info for the attackers
     */
    void calculateInfoForAttackers() noexcept;

    /**
     * Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

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

    void calculateInfoForBlocker () noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_ATTACKING_PASS_PLAY_H
