//
// Created by timovdk on 4/28/20.
//

#ifndef RTT_PENALTYTHEM_H
#define RTT_PENALTYTHEM_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class PenaltyThem : public Play {
   public:
    /**
     * Constructor that initializes roles with test roles
     */
    PenaltyThem();

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(PlayEvaluator &playEvaluator) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World*) noexcept override {};

    /**
     * Gets the play name
     */
    const char* getName() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_PENALTYTHEM_H
