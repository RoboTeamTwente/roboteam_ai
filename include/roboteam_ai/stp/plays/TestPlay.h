//
// Created by timovdk on 3/10/20.
//

#ifndef RTT_TESTPLAY_H
#define RTT_TESTPLAY_H

#include "stp/Play.hpp"

namespace rtt::ai::stp {

class TestPlay : public Play {
   public:
    /**
     * Constructor that initializes roles with test roles
     */
    TestPlay();

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(world::World* world) noexcept override;

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
};
}  // namespace rtt::ai::stp

#endif  // RTT_TESTPLAY_H
