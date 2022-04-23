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
                                           const world::World *world);

    /**
     * Makes a wall if not ready done, saves it in calculatedWallPositions and deals the index
     * @param index Index of the wall position (do unique positions)
     * @param amountDefenders Amount of defenders the wall is made of
     * @param field
     * @param world
     * @return Vector2 position of that index in the wall
     */
    static Vector2 getWallPosition(int index, int amountDefenders, const world::Field &field, world::World *world);

    /**
     * Projects a position outside of the defense area onto the line between two given points
     * @param field The current field
     * @param position The position to be projected outside the defense area
     * @param p1 First point on the line
     * @param p2 Second point on the line
     * @param margin The distance that the position should have from the defense area
     * @return A position outside of the defense area on the given line. If there are two intersections with the defense area,
     * the returned point will be at the intersection closest to p1. If no point outside the defense area can be found, the original position is returned
     */
    static Vector2 ProjectPositionOutsideDefenseAreaOnLine(const world::Field &field, Vector2 position, Vector2 p1, Vector2 p2, double margin);

    /**
     * Projects a position into the field on a line between two given points, if possible.
     * @param field The current field
     * @param position The position to be projected into the field area
     * @param p1 First point on the line
     * @param p2 Second point on the line
     * @param margin The margin with which the field is expanded/shrunk (positive margin => larger field)
     * @return A position inside the field on the given line. If no such position is found, the original position is returned.
     */
    static Vector2 ProjectPositionIntoFieldOnLine(const world::Field &field, Vector2 position, Vector2 p1, Vector2 p2, double margin);

    /**
     * Projects a position to a valid position on a line between two given points. A valid position is defined as in the field and outside of the defense area.
     * Note that this function does not account for altered rules for the keeper/ball_placer etc.
     * @param position The position to be projected to a valid point
     * @param p1 First point on the line
     * @param p2 Second point on the line
     * @param defenseAreaMargin The distance that the position should have from the defense area
     * @param fieldMargin The margin with which the field is expanded/shrunk (positive margin => larger field)
     * @return A position outside of the defense area, inside the field, on the given line.
     * If no such position can be found, a position is returned that is projected into the field, and out of the defense area.
     * (So this position is valid, but not on the given line)
     */
    static Vector2 ProjectPositionToValidPointOnLine(const world::Field &field, Vector2 position, Vector2 p1, Vector2 p2, double defenseAreaMargin, double fieldMargin);
};
}  // namespace rtt::ai::stp
#endif  // RTT_POSITIONCOMPUTATIONS_H
