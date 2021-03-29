//
// Created by maxl on 19-03-21.
//

#ifndef RTT_GENERALIZATIONCONSTANTS_H
#define RTT_GENERALIZATIONCONSTANTS_H

#include <roboteam_utils/Grid.h>

namespace rtt::ai::stp::gen {
    /**
     * Struct that is used to store computations made with this module.
     * Save computations here that are usable for each robot for a position.
     *       DO NOT save values specific to a robot in here (like TimeToPosition).
     * If a position's score for a specific evaluation already had been computed in the tick, it will
     * use that value instead of recomputing it. If it was not computed yet, it will compute and save it.
     *
     * @memberof scoreOpen : uint8_t score for the Openness of a position -> evaluations/position/OpennessEvaluation
     * @memberof scoreLineOfSight : uint8_t score for the LineOfSight to a position from a position -> ../LineOfSightEvaluation
     * @memberof scoreGoalShot : uint8_t score for the Goal Shot opportunity for a position -> ../GoalShotEvaluation
     */
    struct PositionScores {
        std::optional<double> scoreOpen;
        std::optional<double> scoreLineOfSight;
        std::optional<double> scoreGoalShot;
    };

    /**
     * Combination of weights for each of the scores.
     * This will be used to determine the final score for a robot for a position.
     * All weights will be multiplied with the corresponding score and then normalized.
     *
     * @memberof weightOpen for scoreOpen
     * @memberof weightLineOfSight for scoreLineOfSight
     * @memberof weightGoalShot for scoreGoalShot
     */
    struct ScoreProfile{
        double weightOpen;
        double weightLineOfSight;
        double weightGoalShot;
    };

    /**
     * Structure with a position and its score
     * @memberof position Vector2 coordinates of a position
     * @memberof score The score for said position
     */
    struct ScoredPosition{
        Vector2 position;
        uint8_t score;
    };

    /**
     * Generalized Position Profiles to be used in plays.
     * They consist of a generalized weight combination.
     */
    constexpr ScoreProfile SafePosition = {1,1,0};
    constexpr ScoreProfile OffensivePosition = {1,0.5,0.5};
    constexpr ScoreProfile GoalShootPosition = {0, 0.5,1};

    /**
     * Generalized Grids to be used in plays
     * These positions are meant to be used across plays to lower computations when computing which
     * play is best by reducing the amount of unique position that need to be evaluated.
     */
    inline static Grid gridRightTop = Grid(3, 0, 2.5, 2.2, 5, 5);
    inline static Grid gridRightMid = Grid(3, -1.5, 2.5, 3, 5, 5);
    inline static Grid gridRightBot = Grid(3, -2.5, 2.5, 2.2, 5, 5);
    inline static Grid gridMidFieldTop = Grid(-1, 0, 2, 2.2, 5, 5);
    inline static Grid gridMidFieldMid = Grid(-1, -1.5, 2, 3, 5, 5);
    inline static Grid gridMidFieldBot = Grid(-1, -2.5, 2, 2.2, 5, 5);
}
#endif //RTT_GENERALIZATIONCONSTANTS_H
