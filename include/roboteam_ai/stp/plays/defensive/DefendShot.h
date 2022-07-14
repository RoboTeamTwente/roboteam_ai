//
// Created by agata on 14/01/2022.
//

#ifndef RTT_DEFENDSHOT_H
#define RTT_DEFENDSHOT_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * DefendShot Play is executed when the opponent has or is close to the ball and on our side of the field.
 * In this case they most likely will try to score. Some defenders defend the goal by blocking the path between enemy
 * robots and the goal. Other defenders block other enemy robots to avoid passes to them.
 */
class DefendShot : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    DefendShot();

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

    /**
     * Gets the play name
     */
    const char* getName() override;

    /**
     * Check if play should end
     */
    bool shouldEndPlay() noexcept override;

   protected:
    /**
     * Calculates info for the wallers
     */
    void calculateInfoForWallers(bool shouldIncludeBallBlocker) noexcept;

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

#endif  // RTT_DEFENDSHOT_H
