//
// Created by jordi on 01-05-20.
//

#ifndef RTT_TIMEOUT_H
#define RTT_TIMEOUT_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

    class TimeOut : public Play {
    public:
        /**
         * Constructor that initializes roles with roles that are necessary for this play
         */
        TimeOut();

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
         * Info that should be calculated but is not necessary for scoring
         */
        void calculateInfoForRoles() noexcept override;

        /**
         * Info that should be calculated for scoring of the play
         */
        void calculateInfoForScoredRoles(world::World*) noexcept override {};

        /**
         * Gets the play name
         */
        const char *getName() override;
    };
}  // namespace rtt::ai::stp::play

#endif  // RTT_TIMEOUT_H
