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
     * Calculates where a robot should stand to prevent the ball from going in the goal
     * @param field The current field
     * @param world The current world
     * @return The position a robot should go to to block the ball (this does not depend on the position of any of our robots)
     */
    static Vector2 getBallBlockPosition(const world::Field &field, const world::World *world);

    /**
     * Calculates a position, near the target position, that is not too close to the ball
     * @param targetPosition The initial target position
     * @param ballPosition The position of the ball
     * @param field The current field
     * @return A position that is not within the min allowed distance to the ball
     */
    static Vector2 calculateAvoidBallPosition(Vector2 targetPosition, Vector2 ballPosition, const world::Field &field);

   private:
    static Vector2 calculatePositionOutsideOfShape(Vector2 ballPos, const world::Field &field, const std::unique_ptr<Shape> &avoidShape);
};
}  // namespace rtt::ai::stp
#endif  // RTT_POSITIONCOMPUTATIONS_H
