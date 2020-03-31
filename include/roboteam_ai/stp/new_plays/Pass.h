//
// Created by jessevw on 17.03.20.
//

#ifndef RTT_PASS_PLAY_H
#define RTT_PASS_PLAY_H

#include <stp/Play.hpp>

namespace rtt::ai::stp::play {

class Pass : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    Pass();

    bool isValidPlayToStart(world_new::World* world) noexcept override;

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
     * Calculates n defensive positions for the roles to defend
     * @param numberOfDefenders
     * @param world
     * @param enemyRobots
     * @return A vector of defend positions
     */
    std::vector<Vector2> calculateDefensivePositions(int numberOfDefenders, world_new::World* world, std::vector<world_new::view::RobotView> enemyRobots);

    /**
     * Gets the play name
     */
    const char* getName() override;

   protected:
    bool shouldRoleSkipEndTactic() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_PASS_PLAY_H
