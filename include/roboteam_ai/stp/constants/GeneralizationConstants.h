//
// Created by maxl on 19-03-21.
//

#ifndef RTT_GENERALIZATIONCONSTANTS_H
#define RTT_GENERALIZATIONCONSTANTS_H

#include <roboteam_utils/Grid.h>

namespace rtt::ai::stp::gen {
    /**
     * Struct that is used to store computations made with this module
     */
    struct PositionScores {
        std::optional<double> scoreOpen;
        std::optional<double> scoreLineOfSight;
        std::optional<double> scoreGoalShot;
    };

    /**
     * Struct of a position profile
     */
    struct ScoreProfile{
        double weightOpen;
        double weightLineOfSight;
        double weightGoalShot;
    };

    /**
     * Structure with a position and its score
     */
    struct ScoredPosition{
        Vector2 position;
        uint8_t score;
    };

    /**
     * Generalized Position Profiles
     */
    inline static ScoreProfile SafePosition = {1,1,0};
    inline static ScoreProfile OffensivePosition = {1,0.5,0.5};
    inline static ScoreProfile GoalShootPosition = {0, 0.5,1};

    /**
     * Generalized Grids
     */
    inline static Grid gridRightTop = Grid(3, 0, 2.5, 2.2, 5, 5);
    inline static Grid gridRightMid = Grid(3, -1.5, 2.5, 3, 5, 5);
    inline static Grid gridRightBot = Grid(3, -2.5, 2.5, 2.2, 5, 5);
    inline static Grid gridMidFieldTop = Grid(-1, 0, 2, 2.2, 5, 5);
    inline static Grid gridMidFieldMid = Grid(-1, -1.5, 2, 3, 5, 5);
    inline static Grid gridMidFieldBot = Grid(-1, -2.5, 2, 2.2, 5, 5);
}
#endif //RTT_GENERALIZATIONCONSTANTS_H
