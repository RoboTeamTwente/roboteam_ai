//
// Created by martin on 14-5-22.
//

#pragma once

#include <variant>

#include "control/positionControl/CollisionDetector.h"
namespace rtt::ai::control {

/**
 * \brief Wrapper container for 2 trajectories and path-score
 * Combining 2 trajectories is an _expensive_ operation, thus it's better to defer that step to the latest possible moment.
 * cutoffIndex indicates where endTrajectory connects with initialTrajectory (i.e how many points are used from the initialTrajectory)
 * Lower score indicated better path (e.g score = 100 is better than score = 150)
 */
struct CombinedTrajectory {
    BB::BBTrajectory2D initialTrajectory;
    BB::BBTrajectory2D endTrajectory;
    int cutoffIndex = 0;
    int score = std::numeric_limits<int>::max();
};

/**
 * Path planning using the BBT algorithm for single robot.
 */
class BBTPathPlanning {
   private:
    static constexpr int INTERMEDIATE_POINTS_PER_CIRCLE = 8;
    static constexpr double ANGLE_BETWEEN_INTERMEDIATE_POINTS = 2 * M_PI / INTERMEDIATE_POINTS_PER_CIRCLE;

    const int robotId;
    const CollisionDetector& collisionDetector;

    double fieldWidth;
    double maxVelocity;
    stp::AvoidObjects avoidObjects;

    /**
     * \brief Calculates the score of given trajectory. Lower score indicated better path (e.g score = 100 is better than score = 150)
     * @param duration How long the path is (i.e. how many path segments there are in the final path)
     * @param collision information about *first* collision on the path
     */
    [[nodiscard]] static int scorePath(int duration, std::optional<Collision> collision);

    /**
     * \brief Find the best trajectory from initial position to the target position that is skewed towards the intermediatePoint.
     * 1) Trajectory is generated from initialPos to intermediatePoint.
     * 2) Trajectory is generated from each path point to the target position.
     * 3) Best trajectory is selected.
     * @param initialPos Initial Position
     * @param initialVel Initial Velocity
     * @param intermediatePoint Is used to generate intermediate trajectory.
     * @param targetPos Target Position
     */
    [[nodiscard]] CombinedTrajectory bestTrajectoryForIntermediatePoint(const Vector2& initialPos, const Vector2& initialVel, const Vector2& intermediatePoint,
                                                                        const Vector2& targetPos) const;

    /**
     * \brief Generates vector of intermediate points
     * @param center Intermediate points are generated around this point
     */
    [[nodiscard]] std::vector<Vector2> generateIntermediatePoints(const Vector2& center) const;

    /**
     * \brief Combines two BBTrajectories2D into one PosVelVector vector
     * @param combineTraj Trajectories to combine
     */
    [[nodiscard]] std::vector<BB::PosVelVector> extractPath(CombinedTrajectory&& combineTraj) const;

   public:
    BBTPathPlanning(double fieldWidth, double maxVelocity, int robotId, const CollisionDetector& collisionDetector);

    /**
     * \brief Computes best trajectory to reach the target position
     * @param initialPos Initial Position
     * @param initialVel Initial Velocity
     * @param targetPos Target Position
     */
    [[nodiscard]] std::vector<BB::PosVelVector> generateNewPath(const Vector2& initialPos, const Vector2& initialVel, const Vector2& targetPos) const;

    /**
     * \brief Updates the path planning with new information about the world
     * @param field Playing filed
     * @param newAvoidObjects New information about objects to avoid
     * @param newMaxVelocity New maximum velocity of the robot
     */
    void updateConstraints(const rtt::world::Field& field, stp::AvoidObjects newAvoidObjects, double newMaxVelocity);
};

}  // namespace rtt::ai::control
