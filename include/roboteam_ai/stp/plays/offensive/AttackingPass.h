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
     * Calculates n defensive positions for the roles to defend
     * @param numberOfDefenders
     * @param world
     * @param enemyRobots
     * @return A vector of defend positions
     */
    std::vector<Vector2> calculateDefensivePositions(int numberOfDefenders, std::vector<world::view::RobotView> enemyRobots);

    /**
     * Gets the play name
     */
    const char* getName() override;

    /**
     * Calculates all info that is necessary for a correct pass
     * The passer will get a position to pass to
     * Receivers will get positions to receive at, of which one will actually intercept the ball once it is close enough
     * @param ball
     */
    void calculateInfoForPass(const world::ball::Ball* ball) noexcept;

    bool isValidPlayToStart(PlayEvaluator *playEvaluator) noexcept;

   private:
    /**
     * Checks if the pass is finished so the play knows whether it should
     * keep this play or move to another play
     * @return true: when ONE of the receivers is closer than 0.08m to the ball
     *         false: when NONE of the receivers is closer than 0.08m to the ball
     */
    [[nodiscard]] bool passFinished() noexcept;

    /**
     * Did the passer shoot or not
     */
    bool passerShot{false};

    void storePlayInfo(gen::PlayInfos& info) noexcept override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_ATTACKING_PASS_PLAY_H
