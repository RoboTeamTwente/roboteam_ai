//
// Created by thijs on 25-5-19.
//

#include "control/numtrees/Collision.h"
#include "world_new/World.hpp"

namespace rtt::ai::control {

std::string Collision::collisionTypeToString() {
    std::string s;
    switch (getCollisionType()) {
        case Collision::ROBOT:
            s = "ROBOT          ";
            break;
        case Collision::BALL:
            s = "BALL           ";
            break;
        case Collision::FIELD:
            s = "FIELD          ";
            break;
        case Collision::DEFENSE_AREA:
            s = "DEFENSE_AREA   ";
            break;
        case Collision::GOAL:
            s = "GOAL   ";
            break;
        case Collision::BALL_PLACEMENT:
            s = "BALL_PLACEMENT    ";
            break;
        case Collision::NO_COLLISION:
            s = "NO COLLISION?!?";
            break;
        default:
            s = "ERROR! CollisionType does not exist";
    }
    return s;
}

const std::optional<world_new::view::RobotView> Collision::getCollisionRobot() const { return collisionRobot; }

void Collision::setCollisionRobot(const world_new::view::RobotView robot, double distance) {
    type = ROBOT;
    collisionRobot = robot;
    setCollision(distance);
}

void Collision::setCollision(double distance) {
    isCollision = true;
    collisionRadius = distance;
}

void Collision::setCollisionBall(const world_new::view::BallView ball, double distance) {
    type = BALL;
    collisionBall = ball;
    setCollision(distance);
}

void Collision::setFieldCollision(const Vector2 &collisionPos, double distance) {
    type = FIELD;
    fieldCollision = collisionPos;
    setCollision(distance);
}

void Collision::setDefenseAreaCollision(const Vector2 &collisionPos, double distance) {
    type = DEFENSE_AREA;
    Collision::defenseAreaCollision = collisionPos;
    setCollision(distance);
}

void Collision::setGoalCollision(const Vector2 &collisionPos, double distance) {
    type = GOAL;
    Collision::goalCollision = collisionPos;
    setCollision(distance);
}

const Vector2 Collision::collisionPosition() const {
    if (collisionRobot.has_value())
        return collisionRobot->get()->getPos();
    else if (collisionBall.has_value())
        return collisionBall->get()->getPos();
    else if (fieldCollision != Vector2())
        return fieldCollision;
    else if (defenseAreaCollision != Vector2())
        return defenseAreaCollision;
    else
        return {};
}

const Collision::CollisionType Collision::getCollisionType() const { return type; }

const Vector2 &Collision::getCollisionDefenseAreaPos() const { return defenseAreaCollision; }

const Vector2 &Collision::getCollisionFieldPos() const { return fieldCollision; }

const std::optional<world_new::view::BallView> Collision::getCollisionBall() const { return collisionBall; }

const Vector2 &Collision::getCollisionGoalPos() const { return goalCollision; }

const Vector2 &Collision::getCollisionBallPlacement() const { return ballPlacementCollision; }

void Collision::setBallPlacementCollision(const Vector2 &collisionPos, double distance) {
    type = BALL_PLACEMENT;
    Collision::ballPlacementCollision = collisionPos;
    setCollision(distance);
}

}  // namespace rtt::ai::control
