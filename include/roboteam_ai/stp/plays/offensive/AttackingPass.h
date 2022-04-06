//
// Created by jessevw on 17.03.20.
//

#ifndef RTT_ATTACKING_PASS_PLAY_H
#define RTT_ATTACKING_PASS_PLAY_H

#include <roboteam_utils/Grid.h>

#include "stp/Play.hpp"

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
    void calculateInfoForScoredRoles(world::World*) noexcept override;

    /**
     * Gets the play name
     */
    const char* getName() override;

    /**
     * Check if play should end. True if pass arrived or ball is not moving anymore after pass
     */
    bool shouldEndPlay() noexcept override;

    /**
     *
     */
    void storePlayInfo(gen::PlayInfos& info) noexcept override;

   private:
    /**
     * Return true if passer is done with KickAtPos tactic
     */
    bool ballKicked();

    /**
     * Calculate which robots could receive a pass, and calculate the pass location based on that
     */
    gen::ScoredPosition calculatePassLocation(world::World* world);

    /**
     * The location the ball will be passed to. Calculated once for each time this play is run
     */
    std::optional<Vector2> passLocation = std::nullopt;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_ATTACKING_PASS_PLAY_H
