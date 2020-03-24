//
// Created by timovdk on 3/10/20.
//

#ifndef RTT_TESTPLAY_H
#define RTT_TESTPLAY_H

#include <stp/Play.hpp>

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
     * @param world World to get the score for (world_new::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(world_new::World *world) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    void assignRoles() noexcept override;

    bool isValidPlayToStart(world_new::World *world) noexcept override;

    bool isValidPlayToKeep(world_new::World *world) noexcept override;

    void calculateInfoForRoles() noexcept override;

    bool shouldRoleSkipEndTactic() override;
};
}  // namespace rtt::ai::stp

#endif  // RTT_TESTPLAY_H
