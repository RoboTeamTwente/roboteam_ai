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

        double maxVel();
        double maxAcc();
        double maxDec();

        explicit PathPoint();

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
