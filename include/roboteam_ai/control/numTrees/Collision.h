//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_COLLISION_H
#define ROBOTEAM_AI_COLLISION_H

#include <memory>
#include <roboteam_utils/Vector2.h>
#include <utilities/GameStateManager.hpp>
#include "world/Robot.h"
#include "world/Ball.h"

namespace rtt {
namespace ai {
namespace control {

/**
 * Another general thing tbh
 * the standard (STL) uses snake_case for everything
 * It's a very clean convention and I suggest you stick with it
 * I try and always comply with the standard when I develop
 * I know C++ does not really have "standard" converntions
 * However you can look at the STL when developing
 * However, all classes in the STL are snake_case
 * When defining your own classes I like to stick to
 * PascalCase, where all members are snake_case (like Rust's)
 * enforced style
 */

class Collision {

    public:
        enum CollisionType : short {
          ROBOT,
          BALL,
          FIELD,
          DEFENSE_AREA,
          GOAL,
          BALL_PLACEMENT,
          NO_COLLISION
        };
        std::string collisionTypeToString();
    private:
        CollisionType type;

        world::Robot::RobotPtr collisionRobot;
        world::Ball::BallPtr collisionBall;
        Vector2 fieldCollision;
        Vector2 defenseAreaCollision;
        Vector2 goalCollision;
        Vector2 ballPlacementCollision;


    public:
        /**
         * For consistency purposes this shouldn't have been defined
         * in the class
         * 
         * Also in general, for initialization, use {} instead of ()
         * {} prevents implicit conversions to other types
         * 
         * int x = 5.0; // valid, even though 5.0 is a double
         * int x( 5.0 ); // valid, even though 5.0 is a double
         * int x{ 5.0 }; // compiler error, double cannot be converted to int in initializer list
         */
        Collision() : type(NO_COLLISION), isCollision(false), collisionRadius(0.0) {
            collisionRobot = std::make_shared<world::Robot>(world::Robot());
            collisionBall = std::make_shared<world::Ball>(world::Ball());
            fieldCollision = Vector2();
            defenseAreaCollision = Vector2();
            goalCollision = Vector2();
            ballPlacementCollision = Vector2();
        }

        /**
         * I've also mentioned this before I think
         * But std::shared_ptr is considered "bad", due to
         * the runtime over caused
         * In this use case you shoudl really be using std::unique_ptr
         * Your ownership model right now is ehh, and unique_ptr
         * would allow you to clean that up very much
         * a single-owner ownership model is not only 
         * considered good practice, it can legitemately improve your
         * performance
         */

        const world::Robot::RobotPtr &getCollisionRobot() const;
        void setCollisionRobot(const world::Robot::RobotPtr &robot, double distance);

        const world::Ball::BallPtr &getCollisionBall() const;
        void setCollisionBall(const world::Ball::BallPtr &ball, double distance);

        const Vector2 &getCollisionFieldPos() const;
        void setFieldCollision(const Vector2 &collisionPos, double distance);

        const Vector2 &getCollisionDefenseAreaPos() const;
        void setDefenseAreaCollision(const Vector2 &collisionPos, double distance);

        const Vector2 &getCollisionGoalPos() const;
        void setGoalCollision(const Vector2 &collisionPos, double distance);

        const Vector2 &getCollisionBallPlacement() const;
        void setBallPlacementCollision(const Vector2 &collisionPos, double distance);

        bool isCollision;
        double collisionRadius;
        const Vector2 collisionPosition() const;
        const CollisionType getCollisionType() const;

    void setCollision(double distance);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_COLLISION_H
