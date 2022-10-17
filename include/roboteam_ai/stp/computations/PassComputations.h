//
// Created by maxl on 11-02-21.
//

#ifndef RTT_PASSCOMPUTATIONS_H
#define RTT_PASSCOMPUTATIONS_H

#include <roboteam_utils/LineSegment.h>
#include <stp/constants/GeneralizationConstants.h>

#include "roboteam_utils/Grid.h"
#include "world/Field.h"
#include "world/World.hpp"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp {

/**
 * Struct to hold relevant information for passing
 */
struct PassInfo {
    int keeperId = -1;
    int passerId = -1;
    int receiverId = -1;
    Vector2 passLocation;
    uint8_t passScore = 0;
};

}  // namespace rtt::ai::stp
namespace rtt::ai::stp::computations {

class PassComputations {
   public:
    /**
     * Calculates which robot should pass where, and which robot should receive it
     * @param profile the profile to be used when scoring the pass location
     * @param world the current world state
     * @param field the current field
     * @param keeperCanPass indicate whether the keeper can pass and be passed to
     * @return a PassInfo struct which contains the relevant information needed to complete the pass
     */
    static PassInfo calculatePass(gen::ScoreProfile profile, const world::World* world, const world::Field& field, bool keeperCanPass = false);

    /**
     * Scores a given pass based on how likely it is to score from the passLocation, adjusted for the riskiness of the pass
     * @param passInfo The passInfo of the pass to be scored
     * @param world pointer to world
     * @param field the current field
     * @return the score of the pass (0 - 255)
     */
    static uint8_t scorePass(PassInfo passInfo, const world::World* world, const rtt::world::Field& field);

   private:
    /**
     * Gets the grid of points containing all possible pass locations
     * @param field the current field
     * @return a Grid class containing a vector of vectors, which in turn contain all possible pass locations
     */
    static Grid getPassGrid(const world::Field& field);

    /**
     * Indicates whether the given point 1) a valid point to pass to in terms of ssl-rules and 2) whether it is feasible ot pass there
     * @param point the point to check for validity
     * @param ballLocation the current ball location
     * @param possibleReceiverLocations the locations of all robots that could receive the ball
     * @param passerLocation the location of the robot that will pass the ball
     * @param field the current field
     * @param world the current world
     * @return bool indicating whether this point is (likely) possible to pass to
     */
    static bool pointIsValidPassLocation(Vector2 point, Vector2 ballLocation, const std::vector<Vector2>& possibleReceiverLocations, Vector2 passerLocation,
                                         const world::Field& field, const world::World* world);

    /**
     * Determines which robot should be the keeper (either previous keeper, or robot closest to goal if there was no keeper)
     * @param possibleRobots vector of robots that could become keeper
     * @param world current world
     * @param field current field
     * @return Id of robot that should become keeper
     */
    static int getKeeperId(const std::vector<world::view::RobotView>& possibleRobots, const world::World* world, const world::Field& field);

    /**
     * Determines which robot should be the passer (the robot closest to the ball)
     * @param possibleRobots vector of robots that could become passer
     * @param world current world
     * @param field current field
     * @return Id of robot that should become passer
     */
    static int getPasserId(Vector2 ballLocation, const std::vector<world::view::RobotView>& possibleRobots, const world::World* world);

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
