//
// Created by Alexander on 11-05-2022
//

#ifndef RTT_INTERCEPTBALL_H
#define RTT_INTERCEPTBALL_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class InterceptBall : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    InterceptBall();

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
     * Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World* world) noexcept override{};

   protected:
    /**
     * Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * Calculates info for the keeper
     */
    void calculateInfoForKeeper() noexcept;


    /**
     * Check if play should end. True when InterceptBaller role is finished.
     */
    bool shouldEndPlay() noexcept override;

    /**
     * Gets the play name
     */
    const char* getName() override;
};

}  // namespace rtt::ai::stp::play

#endif  // RTT_INTERCEPTBALL_H
