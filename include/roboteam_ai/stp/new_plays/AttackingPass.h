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
     * Calculates n defensive positions for the roles to defend
     * @param numberOfDefenders
     * @param world
     * @param enemyRobots
     * @return A vector of defend positions
     */
    std::vector<Vector2> calculateDefensivePositions(int numberOfDefenders, world::World* world, std::vector<world::view::RobotView> enemyRobots);

    /**
     * Gets the play name
     */
    const char* getName() override;

    /**
     * Checks if this is a valid play to keep
     * @param world
     * @return true if we can keep this play, false if we cannot
     */
    [[nodiscard]] bool isValidPlayToKeep(world::World* world) noexcept override;

   protected:
    /**
     * Checks whether this role should skip the end tactic in its state machine
     * @return whether to skip the end tactic
     */
    bool shouldRoleSkipEndTactic() override;

    /**
     * Calculates the pass location
     * @return a pair of the pass location and the score of that location
     * The score is used to decide to which pass location to pass when there are more receivers
     */
    std::pair<Vector2, double> calculatePassLocation(Grid searchGrid) noexcept;

   private:
    /**
     * Checks if the pass is finished so the play knows whether it should
     * keep this play or move to another play
     * @return true: when ONE of the receivers is closer than 0.08m to the ball
     *         false: when NONE of the receivers is closer than 0.08m to the ball
     */
    [[nodiscard]] bool passFinished() noexcept;

    /**
     * Called every time the .initialize() is called on a play,
     * runs exactly once at the start of this play when this play is picked to be executed
     */
    void onInitialize() noexcept override;

    /**
     * Position that the passer will pass to
     */
    Vector2 passingPosition;

    /**
     * Did the passer shoot or not
     */
    bool passerShot{false};

    /**
     * Two receive locations with their scores.
     * The passer will shoot to the highest scoring position
     */
    std::pair<Vector2, double> receiverPositionLeft{};
    std::pair<Vector2, double> receiverPositionRight{};

    /**
     * The two grids that are used to calculate pass locations within it.
     * In this case the grids are on their side, one on the left and one on the right
     */
    Grid gridLeft = Grid(0.15 * field.getFieldWidth(), 0, 3, 2.5, 5, 5);
    Grid gridRight = Grid(0.15 * field.getFieldWidth(), -2.5, 3, 2.5, 5, 5);
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_ATTACKING_PASS_PLAY_H
