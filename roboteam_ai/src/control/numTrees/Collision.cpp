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
    case Collision::GOAL: s = "GOAL   ";
        break;
    case Collision::NO_COLLISION: s = "NO COLLISION?!?";
        break;
    default: s = "ERROR! CollisionType does not exist";
    }
    return s;
}

const world::Robot::RobotPtr &Collision::getCollisionRobot() const {
    return collisionRobot;
}

void Collision::setCollisionRobot(const world::Robot::RobotPtr &robot, double distance) {
    type = ROBOT;
    collisionRobot = std::make_shared<world::Robot>(world::Robot(*robot));
    setCollision(distance);
}

void Collision::setCollision(double distance) {
    isCollision = true;
    collisionRadius = distance;
}

    void Collision::setCollisionBall(const world::Ball::BallPtr &ball, double distance) {
    type = BALL;
    collisionBall = std::make_shared<world::Ball>(world::Ball(*ball));
    collisionBall->visible = true;
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
    if (collisionRobot) return collisionRobot->pos;
    else if (collisionBall) return collisionBall->pos;
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

const Vector2 &Collision::getCollisionGoalPos() const {
    return goalCollision;
}



}
}
}
