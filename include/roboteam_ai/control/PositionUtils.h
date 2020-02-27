//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_GENERALPOSITIONCOACH_H
#define ROBOTEAM_AI_GENERALPOSITIONCOACH_H

#include <utilities/Constants.h>
#include "roboteam_utils/Vector2.h"
#include "world/Field.h"
#include <world_new/World.hpp>

namespace rtt::ai::control {
using namespace rtt::ai::world;

class PositionUtils {
   public:
    static Vector2 getPositionBehindBallToGoal(world_new::view::BallView ball, const Field &field, double distanceBehindBall, bool ourGoal);
    static Vector2 getPositionBehindBallToPosition(world_new::view::BallView ball, double distanceBehindBall, const Vector2 &position);
    static Vector2 getPositionBehindPositionToPosition(double distanceBehindBall, const Vector2 &behindPosition, const Vector2 &toPosition);
    static bool isRobotBehindBallToGoal(world_new::view::BallView ball, const Field &field, double distanceBehindBall, bool ourGoal, const Vector2 &robotPos, double angleMargin = 0.15);
    static bool isRobotBehindBallToPosition(world_new::view::BallView ball, double distanceBehindBall, const Vector2 &position, const Vector2 &robotPosition, double angleMargin = 0.15);
    static std::vector<Vector2> getPenaltyPositions(const Field &field, int number);
    static std::vector<Vector2> getFreeKickPositions(world_new::view::BallView ball, const Field &field, int number);
    static std::vector<Vector2> getDefendFreeKick(world_new::view::BallView ball, const Field &field, int number);
    static std::vector<Vector2> getDefendPenaltyPositions(const Field &field, int number);
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_GENERALPOSITIONCOACH_H
