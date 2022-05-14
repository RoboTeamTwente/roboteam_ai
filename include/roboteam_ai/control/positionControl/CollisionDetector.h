//
// Created by martin on 11-5-22.
//

#pragma once

#include <optional>
#include <span>
#include <vector>

#include "control/positionControl/BBTrajectories/Trajectory2D.h"
#include "utilities/Constants.h"
#include "world/Field.h"
#include "world/FieldComputations.h"

namespace rtt::ai::control {

struct BallObstacle {
    Vector2 position;
    Vector2 velocity;
};

struct RobotObstacle {
    Vector2 position;
    Vector2 velocity;
    int robotId;
};

struct Obstacles {
    BallObstacle ball;
    std::vector<RobotObstacle> robotsThem;
    std::vector<RobotObstacle> robotsUs;
};

struct Collision {
    Vector2 position;
    double time;
};

using PathPoints = std::span<const Vector2>;

class CollisionDetector {
   private:
    static constexpr double SCAN_RANGE = 2.0;
    static constexpr double TIME_STEP = 0.1;
    static constexpr double TIMELINE_SIZE = SCAN_RANGE / TIME_STEP;

    static constexpr double MIN_ROBOT_DISTANCE = 3 * ai::Constants::ROBOT_RADIUS_MAX();
    double minBallDistance = 0.0;

    std::vector<Obstacles> timeline;
    std::optional<rtt::world::Field> field;
    [[nodiscard]] static bool isCollision(const Vector2& origin, const Vector2& target, const double& threshold);

    std::optional<Collision> getFirstObjectCollision(PathPoints pathPoints, const int& robotId);
    std::optional<Collision> getFirstFieldCollision(PathPoints pathPoints);
    std::optional<Collision> getFirstDefenseAreaCollision(PathPoints pathPoints);

   public:
    CollisionDetector();
    std::optional<Collision> getFirstCollision(const int& robotId, PathPoints pathPoints);
    void setMinBallDistance(double distance);
    void setField(const std::optional<rtt::world::Field>& field);
    void updatePositions(const std::vector<rtt::world::view::RobotView>& robots, const rtt::world::view::BallView& ball);
};
}  // namespace rtt::ai::control
