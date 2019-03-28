//
// Created by mrlukasbos on 28-3-19.
//

#ifndef ROBOTEAM_AI_PATHPOINT_H
#define ROBOTEAM_AI_PATHPOINT_H

#include <memory>
#include <roboteam_utils/Vector2.h>

namespace rtt {
namespace ai {
namespace control {
// If there is another way to return a shared pointer from an object to itself that is more pretty let me know
struct PathPoint : std::enable_shared_from_this<PathPoint> {
private:
    const double maxV = 2.0;
    const double maxAccAtLowV = 6.1;
    const double maxAccAtHighV = 3.1;
    const double maxDecelleration = 6.1;
public:
    Vector2 currentTarget;  //Either the endPoint or an in between target
    Vector2 finalTarget;    //Always the endPoint
    Vector2 pos;
    Vector2 vel;
    Vector2 acc;

    double maxVel() {
        double distanceRemaining = (finalTarget - pos).length();
        double absoluteMax = sqrt(2.0 * maxAcc() * distanceRemaining) * 0.8;
        return absoluteMax > maxV ? maxV : absoluteMax;
    }

    double maxAcc() {
        return vel.length() > maxV * 0.5 ?
               maxAccAtHighV :
               maxAccAtLowV - (maxAccAtLowV - maxAccAtHighV) * ((maxV - vel.length()) / maxV);
    }

    double maxDec() {
        return maxDecelleration;
    }
    double t;
    int collisions;
    std::shared_ptr <PathPoint> parent;
    std::vector <std::shared_ptr<PathPoint>> children;
    std::shared_ptr <PathPoint> backTrack(double backTime);
    std::shared_ptr <PathPoint> backTrack(int maxCollisionDiff);
    std::shared_ptr <PathPoint> backTrack(double backTime, int maxCollisionDiff);
    void addChild(std::shared_ptr <PathPoint> &newChild);
    void addChildren(std::vector <std::shared_ptr<PathPoint>> &newChildren);
    bool isCollision(Vector2 target, double distance);
    bool branchHasTarget(const Vector2 &target);
    bool anyBranchHasTarget(const Vector2 &target);
    bool anyChildHasTarget(const Vector2 &target);
    bool anyParentHasTarget(const Vector2 &target);

};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_PATHPOINT_H
