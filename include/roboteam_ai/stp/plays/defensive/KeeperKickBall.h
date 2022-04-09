//
// Created by Alexander on 29/01/2022.
//

#ifndef RTT_KEEPERKICKBALL_H
#define RTT_KEEPERKICKBALL_H

#include "stp/Play.hpp"

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
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(PlayEvaluator& playEvaluator) noexcept override;

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

    std::optional<Vector2> passLocation = std::nullopt;

    Vector2 calculatePassLocation(world::World* world);
    bool ballKicked();
    bool shouldEndPlay() noexcept override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_KEEPERKICKBALL_H
