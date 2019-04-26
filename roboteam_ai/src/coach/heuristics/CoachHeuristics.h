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

    static const double CLOSE_TO_GOAL_WEIGHT;
    static const double SHOT_AT_GOAL_WEIGHT;
    static const double PASS_LINE_WEIGHT;
    static const double DISTANCE_TO_OPPONENTS_WEIGHT;
    static const double MAX_INTERCEPT_ANGLE; // Maximum angle to check for whether a opponent can intercept the ball
public:
    static double calculateCloseToGoalScore(const Vector2& position);
    static double calculateShotAtGoalScore(const Vector2& position, const WorldData &world);
    static double calculatePassLineScore(const Vector2& position, const WorldData &world);
    static double calculateBehindBallScore(const Vector2& position, const WorldData &world);


    /// Currently not implemented, but might be again later
    static double calculateDistanceToOpponentsScore(const Vector2 &position, const WorldData& world);
};

}
}
}

#endif //ROBOTEAM_AI_COACHHEURISTICS_H
