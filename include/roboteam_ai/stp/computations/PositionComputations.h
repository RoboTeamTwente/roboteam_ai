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
        static Vector2 determineMidfielderPosition(const Grid &searchGrid, const rtt_world::Field &field, rtt_world::World *world);

        /**
         * Determine the best position for a midfielder
         * @param searchGrid the grid you want to choose a position from
         * @param field
         * @param world
         * @return a midfielder position
         */
        static Vector2 determineOpenPosition(const Grid &searchGrid, const rtt_world::Field &field, rtt_world::World *world, bool clearPath = false,  double shotMargin = control_constants::ROBOT_CLOSE_TO_POINT);

        /**
         * Calculates the pass location
         * @return a pair of the pass location and the score of that location
         * The score is used to decide to which pass location to pass when there are more receivers
         */
        static std::pair<Vector2, double> determineGoalShotLocation(const Grid &searchGrid, const rtt::world::Field &field, rtt::world::World *world);
    };
} // namespace rtt::ai::stp::computations
#endif //RTT_POSITIONCOMPUTATIONS_H
