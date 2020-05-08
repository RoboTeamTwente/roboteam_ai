#ifndef RTT_BALLPLACEMENTUS_H
#define RTT_BALLPLACEMENTUS_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

    class BallPlacementUs : public Play {
    public:
        /**
         * Constructor that initializes roles with roles that are necessary for this play
         */
        BallPlacementUs();

        /**
         * Gets the score for the current play
         *
         * On the contrary to isValidPlay() this checks how good the play actually is
         * return in range of 0 - 100
         *
         * @param world World to get the score for (world_new::World::instance())
         * @return The score, 0 - 100
         */
        uint8_t score(world_new::World* world) noexcept override;

        /**
         * Assigns robots to roles of this play
         */
        Dealer::FlagMap decideRoleFlags() const noexcept override;

        /**
         * Calculates info for the roles
         */
        void calculateInfoForRoles() noexcept override;

        /**
         * Gets the play name
         */
        const char* getName() override;

    protected:
        bool shouldRoleSkipEndTactic() override;
    };
}  // namespace rtt::ai::stp::play

#endif // RTT_BALLPLACEMENTUS_H