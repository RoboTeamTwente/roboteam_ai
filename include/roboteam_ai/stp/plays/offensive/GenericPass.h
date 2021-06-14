//
// Created by timovdk on 5/20/20.
//

#ifndef RTT_GENERICPASS_H
#define RTT_GENERICPASS_H

#include <roboteam_utils/Grid.h>

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class GenericPass : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    GenericPass();

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
    const char* getName() override;

    [[nodiscard]] bool isValidPlayToKeep(PlayEvaluator &playEvaluator) noexcept override;

   protected:
    /**
     * Calculates the pass location
     * @return a pair of the pass location and the score of that location
     * The score is used to decide to which pass location to pass when there are more receivers
     */
    std::pair<Vector2, double> calculatePassLocation(Grid searchGrid) noexcept;

    /**
     * Calculates all info that is necessary for a correct pass
     * The passer will get a position to pass to
     * Receivers will get positions to receive at, of which one will actually intercept the ball once it is close enough
     * @param ball
     */
    void calculateInfoForPass(const world::ball::Ball* ball) noexcept;

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
    void onInitialize() noexcept;

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
    gen::ScoredPosition receiverPositionLeft{};
    gen::ScoredPosition receiverPositionRight{};

    /**
     * The two grids that are used to calculate pass locations within it.
     * In this case the grids are on their side, one on the left and one on the right
     */
    Grid gridLeft = Grid(0, 0, 3, 2.5, 5, 5);
    Grid gridRight = Grid(0, -2.5, 3, 2.5, 5, 5);
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_GENERICPASS_H
