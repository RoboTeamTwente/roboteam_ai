//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_COLLISION_H
#define ROBOTEAM_AI_COLLISION_H

#include <memory>
#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include "roboteam_ai/src/world/Robot.h"
#include "roboteam_ai/src/world/Ball.h"

namespace rtt {
namespace ai {
namespace control {

class Collision {

    public:
        enum CollisionType : short {
          ROBOT,
          BALL,
          FIELD,
          DEFENSE_AREA,
          NO_COLLISION
        };
        std::string collisionTypeToString();
    private:
        CollisionType type;

        world::Robot collisionRobot = {};
        world::Ball collisionBall = {};
        Vector2 fieldCollision = {};
        Vector2 defenseAreaCollision = {};


    public:

        Collision() : type(NO_COLLISION), isCollision(false), collisionRadius(0.0) { }

        const world::Robot &getCollisionRobot() const;
        void setCollisionRobot(const world::Robot::RobotPtr &robot, double distance);
        const world::Ball &getCollisionBall() const;
        void setCollisionBall(const world::Ball &ball, double distance);
        const Vector2 &getFieldCollision() const;
        const Vector2 &getDefenseAreaCollision() const;
        void setFieldCollision(const Vector2 &collisionPos, double distance);
        void setDefenseAreaCollision(const Vector2 &collisionPos, double distance);

        bool isCollision;
        double collisionRadius;
        const Vector2 collisionPosition() const;
        const CollisionType getCollisionType() const;
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_COLLISION_H
