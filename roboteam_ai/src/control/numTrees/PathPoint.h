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

class PathPoint {
    private:
        using PathPointer = std::shared_ptr<PathPoint>;

        PathPointer parent = {};
    public:
        void setParent(const PathPointer &parent);
        void setChildren(const std::vector<PathPointer> &children);
    public:
        const PathPointer &getParent() const;
        const std::vector<PathPointer> &getChildren() const;
    private:
        std::vector<PathPointer> children = {};

    public:
        explicit PathPoint() = default;

        Vector2 currentTarget;  //Either the endPoint or an in between target
        Vector2 finalTarget;    //Always the endPoint
        Vector2 pos;
        Vector2 vel;
        Vector2 acc;

        double maxVelocity() { return rtt::ai::GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel; }
        double maxAcceleration() { return Constants::MAX_ACC_UPPER(); }
        double maxDeceleration() { return Constants::MAX_DEC_UPPER(); }
        double maxVel();

        double t = 0;
        int collisions = 0;


        //backtracking
        static const PathPointer &backTrack(const PathPointer &point, double backTime);
        static const PathPointer &backTrack(const PathPointer &point, int maxCollisionDiff);
        static const PathPointer &backTrack(const PathPointer &point, double backTime, int maxCollisionDiff);
        static const std::vector<PathPoint> backTrackPath(PathPointer start, const PathPointer &root);

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
