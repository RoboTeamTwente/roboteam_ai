//
// Created by maxl on 29-03-21.
/// THIS IS MEANT AS A TEMPLATE FOR MAKING PLAYS. DO NOT ADD THIS FILE TO CMAKE.
//

#ifndef RTT_PLAYTEMPLATE_H
#define RTT_PLAYTEMPLATE_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

    class PlayTemplate : public Play {
    public:
        /**
         * Constructor that initializes roles with roles that are necessary for this play
         */
        PlayTemplate();

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
        const char *getName() override;

        /**
         * Optional function to save information for the next play
         * @param info Map-Struct to save info in
         */
        void storePlayInfo(gen::PlayInfos &info) noexcept override;

        /**
         * Optional function to force end a play
         * @return is play should end
         */
        bool shouldEndPlay() noexcept override;
    };
}  // namespace rtt::ai::stp::play

#endif //RTT_PLAYTEMPLATE_H
