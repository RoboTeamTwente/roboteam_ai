//
// Created by maxl on 09-02-21.
//

#ifndef RTT_POSITIONCOMPUTATIONS_H
#define RTT_POSITIONCOMPUTATIONS_H

#include <roboteam_utils/Arc.h>
#include <roboteam_utils/Circle.h>
#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Line.h>

#include <cmath>
#include <optional>

#include "stp/constants/GeneralizationConstants.h"
#include "utilities/Constants.h"
#include "world/Field.h"
#include "world/FieldComputations.h"
#include "world/World.hpp"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai::stp {

class PositionComputations {
   public:
    /**
     * Determines the location for defenders around the defense area
     * Uses the defence area boundaries and the path from ball to center of goal to find the intersects of circles to
     * find the various positions.
     * @param field
     * @param world
     * @param amountDefenders to be placed
     * @return vector with Vector2 positions for each of the defenders
     */
    static std::vector<Vector2> determineWallPositions(const world::Field &field, const world::World *world, int amountDefenders);

    /**
     * Returns the best scored position from a grid with a profile
     * @param currentPosition The position the robot it currently going to (small biased) if it exists
     * @param searchGrid the area (with points) that should be searched
     * @param profile combination of weights for different factors that should be scored
     * @param field
     * @param world
     * @return the best position within that grid with its score
     */
    static gen::ScoredPosition getPosition(std::optional<rtt::Vector2> currentPosition, const Grid &searchGrid, gen::ScoreProfile profile, const world::Field &field,
                                           world::World *world);

    /**
     * Makes a wall if not ready done, saves it in calculatedWallPositions and deals the index
     * @param index Index of the wall position (do unique positions)
     * @param amountDefenders Amount of defenders the wall is made of
     * @param field
     * @param world
     * @return Vector2 position of that index in the wall
     */
    static Vector2 getWallPosition(int index, int amountDefenders, const world::Field &field, world::World *world);
    static Vector2 ProjectPositionOutsideDefenseAreaOnLine(const world::Field &field, Vector2 position, Vector2 p1, Vector2 p2, double margin);
};
}  // namespace rtt::ai::stp
#endif  // RTT_POSITIONCOMPUTATIONS_H
