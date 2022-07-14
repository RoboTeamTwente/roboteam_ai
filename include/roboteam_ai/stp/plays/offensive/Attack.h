//
// Created by jordi on 24-03-20.
//

#ifndef RTT_ATTACK_H
#define RTT_ATTACK_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class Attack : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    Attack();

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
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

    /**
     * Check if play should end. True when attacker role is finished.
     */
    bool shouldEndPlay() noexcept override;

    /**
     * Gets the play name
     */
    const char* getName() override;

    void calculateInfoForBlocker() noexcept;
};

}  // namespace rtt::ai::stp::play

#endif  // RTT_ATTACK_H
