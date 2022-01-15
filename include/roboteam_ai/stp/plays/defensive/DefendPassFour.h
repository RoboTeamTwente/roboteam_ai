//
// Created by agata on 14/01/2022.
//

#ifndef RTT_DEFENDPASSFOUR_H
#define RTT_DEFENDPASSFOUR_H

#include "stp/Play.hpp"


namespace rtt::ai::stp::play {

/**
 * DefendPass Play is executed when the opponent has or is close to the ball but not necessarily on our side of the field.
 * In this case the opponent most likely will pass to another robot. Our robots will namely block off robots that can
 * be passed to.
 */
class DefendPassFour : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    DefendPassFour();

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(PlayEvaluator& playEvaluator) noexcept;

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
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

    /**
     * Gets the play name
     */
    const char* getName() override;

   protected:
    /**
     * Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * Calculates info for the blockers
     */
    void calculateInfoForBlockers() noexcept;

    /**
     * Calculates info for the keeper
     */
    void calculateInfoForKeeper() noexcept;

    /**
     * Calculates info for the harassers
     */
    void calculateInfoForHarassers() noexcept;

    /**
     * Calculates info for the offenders
     */
    void calculateInfoForOffenders() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_DEFENDPASSFOUR_H
