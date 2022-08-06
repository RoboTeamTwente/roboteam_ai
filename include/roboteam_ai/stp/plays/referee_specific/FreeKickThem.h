//
// Created by jordi on 07-05-20.
//

#ifndef RTT_FREEKICKTHEM_H
#define RTT_FREEKICKTHEM_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class FreeKickThem : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    FreeKickThem();

    /**
     * Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::world::Field &field) noexcept override;

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
    void calculateInfoForScoredRoles(world::World *) noexcept override{};

    /**
     * Gets the play name
     */
    const char *getName() override;

   protected:
    /**
     * Calculates info for the wallers
     */
    void calculateInfoForWallers() noexcept;

    /**
     * Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * Calculates info for the ballBlocker
     */
    void calculateInfoForBlocker() noexcept;

    /**
     * Calculates info for the harasser
     */
    void calculateInfoForHarasser() noexcept;

    /**
     * Calculates info for the keeper
     */
    void calculateInfoForKeeper() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_FREEKICKTHEM_H
