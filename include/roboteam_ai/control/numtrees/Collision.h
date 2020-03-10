//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_COLLISION_H
#define ROBOTEAM_AI_COLLISION_H

#include <roboteam_utils/Vector2.h>
#include <memory>
#include <utilities/GameStateManager.hpp>
#include "world_new/views/BallView.hpp"
#include "world_new/views/RobotView.hpp"

namespace rtt::ai::control {

class Collision {
   public:
    enum CollisionType : short { ROBOT, BALL, FIELD, DEFENSE_AREA, GOAL, BALL_PLACEMENT, NO_COLLISION };
    std::string collisionTypeToString();

   private:
    CollisionType type;

    std::optional<world_new::view::RobotView> collisionRobot;
    std::optional<world_new::view::BallView> collisionBall;
    Vector2 fieldCollision;
    Vector2 defenseAreaCollision;
    Vector2 goalCollision;
    Vector2 ballPlacementCollision;

   public:
    Collision() : type(NO_COLLISION), isCollision(false), collisionRadius(0.0) {
        fieldCollision = Vector2();
        defenseAreaCollision = Vector2();
        goalCollision = Vector2();
        ballPlacementCollision = Vector2();
    }

    [[nodiscard]] const std::optional<world_new::view::RobotView> getCollisionRobot() const;
    void setCollisionRobot(world_new::view::RobotView robot, double distance);

    [[nodiscard]] const std::optional<world_new::view::BallView> getCollisionBall() const;
    void setCollisionBall(world_new::view::BallView ball, double distance);

    [[nodiscard]] const Vector2 &getCollisionFieldPos() const;
    void setFieldCollision(const Vector2 &collisionPos, double distance);

    [[nodiscard]] const Vector2 &getCollisionDefenseAreaPos() const;
    void setDefenseAreaCollision(const Vector2 &collisionPos, double distance);

    [[nodiscard]] const Vector2 &getCollisionGoalPos() const;
    void setGoalCollision(const Vector2 &collisionPos, double distance);

    [[nodiscard]] const Vector2 &getCollisionBallPlacement() const;
    void setBallPlacementCollision(const Vector2 &collisionPos, double distance);

    bool isCollision;
    double collisionRadius;
    [[nodiscard]] const Vector2 collisionPosition() const;
    [[nodiscard]] const CollisionType getCollisionType() const;

    void setCollision(double distance);
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_COLLISION_H
