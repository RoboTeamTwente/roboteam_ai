//
// Created by robzelluf on 3/21/19.
//

#ifndef ROBOTEAM_AI_COACHHEURISTICS_H
#define ROBOTEAM_AI_COACHHEURISTICS_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/world/World.h>

namespace rtt {
namespace ai {
namespace coach {

class CoachHeuristics {
private:
    using WorldData = world::WorldData;
    using Ball = world::Ball;
    using Robot = world::Robot;
    using BallPtr = world::World::BallPtr;
    using RobotPtr = world::World::RobotPtr;

    static const double MAX_DISTANCE_FROM_BALL;
    static const double CLOSE_TO_GOAL_WEIGHT;
    static const double SHOT_AT_GOAL_WEIGHT;
    static const double PASS_LINE_WEIGHT;
    static const double DISTANCE_TO_OPPONENTS_WEIGHT;
    static const double DISTANCE_FROM_CORNER_WEIGHT;
public:
    static double calculateCloseToGoalScore(const Vector2& position);
    static double calculateShotAtGoalScore(const Vector2& position, WorldData world);
    static double calculatePassLineScore(const Vector2& position, WorldData world);
    static double calculateDistanceToOpponentsScore(const Vector2 &position, const WorldData world);
    static double calculateDistanceFromCornerScore(const Vector2& position, const roboteam_msgs::GeometryFieldSize& field);
    static double calculateDistanceFromBallScore(const Vector2& position, roboteam_msgs::GeometryFieldSize& field, roboteam_msgs::WorldBall& ball);
    static double calculatePositionScore(const Vector2& position);
    static double calculatePassScore(const Vector2& position);
};

}
}
}


#endif //ROBOTEAM_AI_COACHHEURISTICS_H
