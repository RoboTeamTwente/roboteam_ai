//
// Created by jordi on 11-05-20.
//

#ifndef RTT_GETBALLPOSSESSION_H
#define RTT_GETBALLPOSSESSION_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class GetBallPossession : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    GetBallPossession();

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * returns a value in range of 0 - 255 based on the factors from in the scoring vector
     *
     * To calculate the score of a play, the current situation (Evaluations) and future positions are taken into account.
     * To reduce computations the calculates for the future positions are saved in an StpInfos map that will be added to
     *  the play with the initialization.
     *
     * @param a StpInfos to store calculated info in
     * @return The score, 0 - 255
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
    void calculateInfoForScoredRoles(world::World* world) noexcept override;

    /**
     * Gets the play name
     */
    const char* getName() override;

   protected:
    void storePlayInfo(gen::PlayInfos& info) noexcept override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_GETBALLPOSSESSION_H
