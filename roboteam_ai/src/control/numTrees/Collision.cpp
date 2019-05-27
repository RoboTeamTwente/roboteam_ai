//
// Created by thijs on 25-5-19.
//

#include "Collision.h"

namespace rtt {
namespace ai {
namespace control {

std::string Collision::collisionTypeToString() {
    std::string s;
    switch (getCollisionType()) {
    case Collision::ROBOT: s = "ROBOT          ";
        break;
    case Collision::BALL: s = "BALL           ";
        break;
    case Collision::FIELD: s = "FIELD          ";
        break;
    case Collision::DEFENSE_AREA: s = "DEFENSE_AREA   ";
        break;
    case Collision::NO_COLLISION: s = "NO COLLISION?!?";
        break;
    }
    return s;
}

const world::Robot::RobotPtr &Collision::getCollisionRobot() const {
    return collisionRobot;
}

void Collision::setCollisionRobot(const world::Robot::RobotPtr &robot, double distance) {
    type = ROBOT;
    collisionRobot = robot->copy();
    isCollision = true;
    collisionRadius = distance;
}

void Collision::setCollisionBall(const world::Ball::BallPtr &ball, double distance) {
    type = BALL;
    collisionBall = ball->copy();
    collisionBall->visible = true;
    isCollision = true;
    collisionRadius = distance;
}

void Collision::setFieldCollision(const Vector2 &collisionPos, double distance) {
    type = FIELD;
    fieldCollision = collisionPos;
    isCollision = true;
    collisionRadius = distance;
}

void Collision::setDefenseAreaCollision(const Vector2 &collisionPos, double distance) {
    type = DEFENSE_AREA;
    Collision::defenseAreaCollision = collisionPos;
    isCollision = true;
    collisionRadius = distance;
}

const Vector2 Collision::collisionPosition() const {
    if (collisionRobot->id != - 1) return collisionRobot->pos;
    else if (collisionBall->visible) return collisionBall->pos;
    else if (fieldCollision != Vector2()) return fieldCollision;
    else if (defenseAreaCollision != Vector2()) return defenseAreaCollision;
    else return {};
}

const Collision::CollisionType Collision::getCollisionType() const {
    return type;
}

const Vector2 &Collision::getCollisionDefenseAreaPos() const {
    return defenseAreaCollision;
}

const Vector2 &Collision::getCollisionFieldPos() const {
    return fieldCollision;
}

const world::Ball::BallPtr &Collision::getCollisionBall() const {
    return collisionBall;
}

}
}
}
