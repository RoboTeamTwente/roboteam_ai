//
// Created by maxl on 09-02-21.
//

#ifndef RTT_POSITIONCOMPUTATIONS_H
#define RTT_POSITIONCOMPUTATIONS_H

#include <roboteam_utils/Arc.h>
#include <roboteam_utils/Line.h>

#include <cmath>
#include <optional>
#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Circle.h>

#include "utilities/Constants.h"
#include "world/Field.h"
#include "world/FieldComputations.h"
#include "world/views/WorldDataView.hpp"


namespace rtt::ai::stp {

    class PositionComputations {
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

    private:
        /**
         * Determine score for the Open at given position
         * @param point Position to calculate from
         * @param world
         * @param scores ref to struct linked to that pos
         * @return score value
         */
        static double determineOpenScore(Vector2 &point, world::World *world, PositionScores &scores);

        /**
         * Determine score for the Line of Sight to the ball at given position
         * @param point Position to calculate from
         * @param world
         * @param scores ref to struct linked to that pos
         * @return score value
         */
        static double determineLineOfSightScore(Vector2 &point, world::World *world, PositionScores &scores);

        /**
         * Determine score for the Visibility of the goal at given position
         * @param point Position to calculate from
         * @param field
         * @param world
         * @param scores ref to struct linked to that pos
         * @return score value
         */
        static double determineGoalShotScore(Vector2 &point, const world::Field &field, world::World *world, PositionScores &scores);

        /**
         * Score a position using the given weights weights for a profile.
         * Will check if the position already has a pre-calculated score (from this tick) then throws the weight over it
         * and sums the scores, resulting in a position scored a particular set of weights.
         * @param profile set of weights of the different factors that determine the score
         * @param position x,y coordinates
         * @param scores reference to scores of said position in map
         * @param world
         * @param field
         * @return score of position including the weights
         */
        static uint8_t getScoreOfPosition(ScoreProfile &profile, Vector2 position, PositionScores &scores, world::World *world,
                                          const world::Field &field);


    public:
        /**
         * Struct with a Vector2 (x,y) position with a score (normalized)
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
        inline static Grid gridRightBot = Grid(3, -2.5, 2.5, 2.2, 5, 5);
        inline static Grid gridMidFieldTop = Grid(-1, 0, 2, 2.2, 5, 5);
        inline static Grid gridMidFieldBot = Grid(-1, -2.5, 2, 2.2, 5, 5);

        /**
         * unordered map of calculated scores of this tick.
         * must be cleared at start of tick
         */
        inline static std::unordered_map<Vector2,PositionScores> calculatedScores{};

        /**
         * Determines the location for defenders around the defense area
         * Uses the defence area boundaries and the path from ball to center of goal to find the intersects of circles to
         * find the various positions.
         * @param field
         * @param world
         * @param amountDefenders to be placed
         * @return vector with Vector2 positions for each of the defenders
         */
        static std::vector<Vector2> determineWallPositions(const world::Field &field, world::World *world, int amountDefenders);

        static ScoredPosition getPosition(const Grid &searchGrid, ScoreProfile profile, const world::Field &field, world::World *world);
    };
} // namespace rtt::ai::stp::computations
#endif //RTT_POSITIONCOMPUTATIONS_H
