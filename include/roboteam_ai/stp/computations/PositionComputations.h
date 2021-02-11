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

#include "utilities/Constants.h"
#include "world/Field.h"
#include "world/FieldComputations.h"
#include "world/views/WorldDataView.hpp"

using Vector2 = rtt::Vector2;
using Angle = rtt::Angle;

namespace rtt::ai::stp::computations {
        namespace rtt_world = rtt::world;
    class PositionComputations {
    public:
        /**
         * Determine the best position for a midfielder
         * @param searchGrid the grid you want to choose a position from
         * @param field
         * @param world
         * @return a midfielder position
         */
        static std::pair<Vector2, double> determineBestOpenPosition(const Grid &searchGrid, const rtt_world::Field &field, rtt_world::World *world);

        /**
         * Determine the best position for a midfielder
         * @param searchGrid the grid you want to choose a position from
         * @param field
         * @param world
         * @return a midfielder position
         */
        static std::pair<Vector2, double> determineBestLineOfSightPosition(const Grid &searchGrid, const rtt_world::Field &field, rtt_world::World *world);

        /**
         * Calculates the pass location
         * @return a pair of the pass location and the score of that location
         * The score is used to decide to which pass location to pass when there are more receivers
         */
        static std::pair<Vector2, double> determineBestGoalShotLocation(const Grid &searchGrid, const rtt::world::Field &field, rtt::world::World *world);

        /**
         * Determine best position using specification of contribution of factors
         * @param search Gridthe grid you want to choose a position from
         * @param field
         * @param world
         * @param factorOpen Factor at which Open specification should count (higher is more important)
         * @param factorLineOfSight Factor at which Line of Sight to the ball specification should count (higher is more important)
         * @param factorVisionGoal Factor at which Visibility of ENEMY goal specification should count (higher is more important)
         * @return Best location with given specifications, Vector2 in first and the score in second
         */
        static std::pair<Vector2, double>
        determineBestLocation(const Grid &searchGrid, const world::Field &field, world::World *world, int factorOpen,
                              int factorLineOfSight, int factorVisionGoal);

        /**
         * Determine score for the Open at given position
         * @param point Position to calculate from
         * @param world
         * @return score value
         */
        static double determineOpenScore(Vector2 point, world::World *world);

        /**
         * Determine score for the Line of Sight to the ball at given position
         * @param point Position to calculate from
         * @param world
         * @return score value
         */
        static double determineLineOfSightScore(Vector2 point, world::World *world);

        /**
         * Determine score for the Visibility of the goal at given position
         * @param point Position to calculate from
         * @param field
         * @param world
         * @return score value
         */
        static double determineGoalShotScore(Vector2 point, world::Field &field, world::World *world);
    };
} // namespace rtt::ai::stp::computations
#endif //RTT_POSITIONCOMPUTATIONS_H
