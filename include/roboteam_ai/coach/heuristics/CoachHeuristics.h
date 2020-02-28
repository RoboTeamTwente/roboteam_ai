//
// Created by robzelluf on 3/21/19.
//

#ifndef ROBOTEAM_AI_COACHHEURISTICS_H
#define ROBOTEAM_AI_COACHHEURISTICS_H

#include <roboteam_utils/Vector2.h>
#include <world/Field.h>
#include <world/World.h>
#include <world/WorldData.h>
#include <include/roboteam_ai/world_new/views/WorldDataView.hpp>

namespace rtt::ai::coach {
using namespace rtt::ai::world;

class CoachHeuristics {
   private:
    using WorldData = world::WorldData;
    using Ball = world::Ball;
    using Robot = world::Robot;
    using BallPtr = world::World::BallPtr;
    using RobotPtr = world::World::RobotPtr;

    static const double CLOSE_TO_GOAL_WEIGHT;
    static const double SHOT_AT_GOAL_WEIGHT;
    static const double PASS_LINE_WEIGHT;
    static const double DISTANCE_TO_OPPONENTS_WEIGHT;
    static const double DISTANCE_TO_US_WEIGHT;
    static const double MAX_INTERCEPT_ANGLE;  // Maximum angle to check for whether a opponent can intercept the ball
    static const double ANGLE_TO_GOAL_WEIGHT;

   public:
    static double calculateCloseToGoalScore(const Field &field, const Vector2 &position);
    static double calculateShotAtGoalScore(const Field &field, const Vector2 &position, world_new::view::WorldDataView world);
    static double calculatePassLineScore(const Vector2 &position, world_new::view::WorldDataView world);
    static double calculateBehindBallScore(const Vector2 &position, world_new::view::WorldDataView world);
    static double calculatePassDistanceToBallScore(const Field &field, const Vector2 &position, world_new::view::WorldDataView world);
    static double calculatePositionDistanceToBallScore(const Field &field, const Vector2 &position, world_new::view::WorldDataView world);
    static double calculateAngleToGoalScore(const Field &field, const Vector2 &position);

    /// Currently not implemented, but might be again later
    static double calculateDistanceToOpponentsScore(const Vector2 &position);
    static double calculateDistanceToClosestTeamMateScore(const Vector2 &position, int thisRobotID = -1);
    static double getClosestOpponentAngleToPassLine(const Vector2 &position, world_new::view::WorldDataView world, double smallestAngle = 999999);
};

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_COACHHEURISTICS_H
