//
// Created by maxl on 11-02-21.
//

#ifndef RTT_PASSCOMPUTATIONS_H
#define RTT_PASSCOMPUTATIONS_H

#include <roboteam_utils/LineSegment.h>
#include <stp/constants/GeneralizationConstants.h>

#include "world/Field.h"
#include "world/World.hpp"

namespace rtt::ai::stp::computations {

class PassComputations {
   public:
    /**
     * Checks if there are given bots within the given tube
     * @param passLine Tube area within to check
     * @param robots Vector of RobotViews which needs to be checked
     * @return True if any of the given robots are inside the given Tube
     */
    static bool pathHasAnyRobots(Line passLine, std::vector<Vector2> robotPositions);

    /**
     * Given a list of possible robot locations to pass to, calculate the best pass location
     * @param ballLocation location of the ball
     * @param robotLocations locations of possible robots to pass to
     * @param passerLocation location of passer
     * @param profile the profile to be used when scoring possible passing locations
     * @param world world
     * @param field field
     * @return Scored pass location
     */
    static gen::ScoredPosition calculatePassLocation(Vector2 ballLocation, const std::vector<Vector2>& robotLocations, Vector2 passerLocation, gen::ScoreProfile profile,
                                                     const rtt::world::World* world, const world::Field& field);

    /**
     * Approximate the time it takes a robot to reach a point
     * @param robotPosition current position of robot
     * @param targetPosition position to calculate travel time to
     * @return approximated time to reach target from position
     */
    static double calculateRobotTravelTime(Vector2 robotPosition, Vector2 targetPosition);

    /**
     * Approximate the time it takes the ball to reach a point
     * @param ballLocation current location of the ball
     * @param passerLocation current location of the passer
     * @param targetPosition position to calculate travel time to
     * @return approximated time for ball to reach target position
     */
    static double calculateBallTravelTime(Vector2 ballLocation, Vector2 passerLocation, Vector2 targetPosition);
};
}  // namespace rtt::ai::stp::computations
#endif  // RTT_PASSCOMPUTATIONS_H
