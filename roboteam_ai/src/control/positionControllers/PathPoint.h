//
// Created by mrlukasbos on 28-3-19.
//

#ifndef ROBOTEAM_AI_PATHPOINT_H
#define ROBOTEAM_AI_PATHPOINT_H

#include <memory>
#include <roboteam_utils/Vector2.h>
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
        Vector2 otherCollision = {};

    public:
        enum CollisionType : short {
          ROBOT,
          BALL,
          OTHER
        };
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
        const Vector2 &getOtherCollision() const {
            return otherCollision;
        }
        void setOtherCollision(const Vector2 &collisionPos, double distance) {
            type = OTHER;
            Collision::otherCollision = collisionPos;
            isCollision = true;
            collisionRadius = distance;
        }

        bool isCollision;
        double collisionRadius;
        const Vector2 collisionPosition() const {
            if (collisionRobot.id != - 1) return collisionRobot.pos;
            else if (collisionBall.exists) return collisionBall.pos;
            else return otherCollision;
        }
        const CollisionType getCollisionType() const {
            return type;
        }

};

// If there is another way to return a shared pointer from an object to itself that is more pretty let me know
class PathPoint : public std::enable_shared_from_this<PathPoint> {
    private:
        double maxV = 2.0;
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

        double t;
        int collisions;
        std::shared_ptr<PathPoint> parent;
        std::vector<std::shared_ptr<PathPoint>> children;
        std::shared_ptr<PathPoint> backTrack(double backTime);
        std::shared_ptr<PathPoint> backTrack(int maxCollisionDiff);
        std::shared_ptr<PathPoint> backTrack(double backTime, int maxCollisionDiff);
        void addChild(std::shared_ptr<PathPoint> &newChild);
        void addChildren(std::vector<std::shared_ptr<PathPoint>> &newChildren);
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
