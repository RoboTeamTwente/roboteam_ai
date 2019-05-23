//
// Created by mrlukasbos on 28-3-19.
//

#ifndef ROBOTEAM_AI_PATHPOINT_H
#define ROBOTEAM_AI_PATHPOINT_H

#include <memory>
#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include "roboteam_ai/src/world/Robot.h"
#include "roboteam_ai/src/world/Ball.h"

namespace rtt {
namespace ai {
namespace control {

class NumTreePosControl;

class Collision {
    private:
        world::Robot collisionRobot = {};
        world::Ball collisionBall = {};
        Vector2 fieldCollision = {};
        Vector2 defenseAreaCollision = {};

    public:
        enum CollisionType : short {
          ROBOT,
          BALL,
          FIELD,
          DEFENSE_AREA,
          NO_COLLISION
        };
        std::string collisionTypeToString() {
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
    private:
        CollisionType type;
    public:

        Collision()
                :isCollision(false), collisionRadius(0.0) { }

        const world::Robot &getCollisionRobot() const {
            return collisionRobot;
        }
        void setCollisionRobot(const world::Robot &robot, double distance) {
            type = ROBOT;
            collisionRobot = robot;
            isCollision = true;
            collisionRadius = distance;
        }
        const world::Ball &getCollisionBall() const {
            return collisionBall;
        }
        void setCollisionBall(const world::Ball &ball, double distance) {
            type = BALL;
            Collision::collisionBall = ball;
            isCollision = true;
            collisionRadius = distance;
        }
        const Vector2 &getFieldCollision() const {
            return fieldCollision;
        }
        const Vector2 &getDefenseAreaCollision() const {
            return defenseAreaCollision;
        }
        void setFieldCollision(const Vector2 &collisionPos, double distance) {
            type = FIELD;
            Collision::fieldCollision = collisionPos;
            isCollision = true;
            collisionRadius = distance;
        }
        void setDefenseAreaCollision(const Vector2 &collisionPos, double distance) {
            type = DEFENSE_AREA;
            Collision::defenseAreaCollision = collisionPos;
            isCollision = true;
            collisionRadius = distance;
        }

        bool isCollision;
        double collisionRadius;
        const Vector2 collisionPosition() const {
            if (collisionRobot.id != - 1) return collisionRobot.pos;
            else if (collisionBall.visible) return collisionBall.pos;
            else if (fieldCollision != Vector2()) return fieldCollision;
            else if (defenseAreaCollision != Vector2()) return defenseAreaCollision;
            else return {};
        }
        const CollisionType getCollisionType() const {
            return type;
        }
};

// If there is another way to return a shared pointer from an object to itself that is more pretty let me know
class PathPoint : public std::enable_shared_from_this<PathPoint> {
    private:
        double maxV = 8.0;
        using PathPointer = std::shared_ptr<PathPoint>;
        double maxAccAtLowV = 6.1;
        double maxAccAtHighV = 3.1;
        double maxDecelleration = 6.1;

    public:
        Vector2 currentTarget;  //Either the endPoint or an in between target
        Vector2 finalTarget;    //Always the endPoint
        Vector2 pos;
        Vector2 vel;
        Vector2 acc;

        double maxVel() {
            double distanceRemaining = (finalTarget - pos).length();
            double absoluteMax = sqrt(2.0*maxAcc()*distanceRemaining)*0.8;
            return absoluteMax > maxV ? maxV : absoluteMax;
        }

        double maxAcc() {
            return vel.length() > maxV*0.5 ?
                   maxAccAtHighV :
                   maxAccAtLowV - (maxAccAtLowV - maxAccAtHighV)*((maxV - vel.length())/maxV);
        }

        double maxDec() {
            return maxDecelleration;
        }

        explicit PathPoint() {
            maxV = GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel;
            maxAccAtLowV = 6.1;
            maxAccAtHighV = 3.1;
            maxDecelleration = 6.1;
        }

        double t = 0;
        int collisions = 0;
        PathPointer parent = {};
        std::vector<PathPointer> children = {};
        PathPointer backTrack(double backTime);
        PathPointer backTrack(int maxCollisionDiff);
        PathPointer backTrack(double backTime, int maxCollisionDiff);
        void addChild(PathPointer &newChild);
        void addChildren(std::vector<PathPointer> &newChildren);
        bool isCollision(const Vector2 &target, double distance);

        bool branchHasTarget(const Vector2 &target);
        bool anyBranchHasTarget(const Vector2 &target);
        bool anyChildHasTarget(const Vector2 &target);
        bool anyParentHasTarget(const Vector2 &target);

};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_PATHPOINT_H
