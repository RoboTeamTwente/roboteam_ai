//
// Created by mrlukasbos on 28-3-19.
//

#include "PathPoint.h"

namespace rtt {
namespace ai {
namespace control {

/// go back in the path until desired time or until Root
std::shared_ptr<PathPoint> PathPoint::backTrack(double backTime) {
    if (! parent)
        return shared_from_this();
    else if (backTime > t)
        return shared_from_this();
    else
        return parent->backTrack(backTime);
}

/// go back in the path until desired collision difference or until Root
std::shared_ptr<PathPoint> PathPoint::backTrack(int maxCollisionDiff) {
    if (! parent)
        return shared_from_this();
    else if (maxCollisionDiff == 0)
        return shared_from_this();
    else
        return parent->backTrack(collisions - parent->collisions);
    if (collisions > parent->collisions)
        return parent->backTrack(maxCollisionDiff - 1);
    else
        return parent->backTrack(maxCollisionDiff);
}

/// go back in the path until desired time, collision difference or until Root
std::shared_ptr<PathPoint> PathPoint::backTrack(double backTime, int maxCollisionDiff) {

    if (! parent)
        return shared_from_this();
    else if (maxCollisionDiff == 0)
        return shared_from_this();
    else if (collisions > parent->collisions)
        return parent->backTrack(maxCollisionDiff - 1);
    else if (backTime > t)
        return shared_from_this();
    else
        return parent->backTrack(backTime, maxCollisionDiff);
}

/// add a child to the current path
void PathPoint::addChild(std::shared_ptr <PathPoint> &newChild) {
    children.push_back(newChild);
}

/// add multiple children to the current path
void PathPoint::addChildren(std::vector <std::shared_ptr<PathPoint>> &newChildren) {
    children.insert(children.end(), newChildren.begin(), newChildren.end());
}

/// check if a branch already has the target
bool PathPoint::branchHasTarget(const Vector2 &target) {

    for (const auto &child : children) {
        if ((child->currentTarget - target).length() < 0.15) {
            return true;
        }
    }
    return false;
}

/// check if ANY branch already has that target
bool PathPoint::anyBranchHasTarget(const Vector2 &target) {
    auto root = backTrack(0.0);
    return root->anyChildHasTarget(target);
}

/// check if ANY child already has that target
bool PathPoint::anyChildHasTarget(const Vector2 &target) {
    for (const auto &child : children) {
        if ((child->currentTarget - target).length() < 0.15 || child->anyChildHasTarget(target)) {
            return true;
        }
    }
    return false;
}

/// check if ANY parent already has that target
bool PathPoint::anyParentHasTarget(const Vector2 &target) {
    if (parent) {
        if ((parent->currentTarget - target).length() < 0.15 || parent->anyParentHasTarget(target)) {
            return true;
        }
    }
    return false;
}

/// check if a collision is occuring
bool PathPoint::isCollision(const Vector2 &target, double distance) {
    return pos.dist2(target) < distance*distance;
}

double PathPoint::maxVel() {
    double distanceRemaining = (finalTarget - pos).length();
    double absoluteMax = sqrt(2.0*maxAcc()*distanceRemaining)*0.8;
    return absoluteMax > maxV ? maxV : absoluteMax;
}

double PathPoint::maxAcc() {
    return vel.length() > maxV*0.5 ?
           maxAccAtHighV :
           maxAccAtLowV - (maxAccAtLowV - maxAccAtHighV)*((maxV - vel.length())/maxV);
}

double PathPoint::maxDec() {
    return maxDecelleration;
}

PathPoint::PathPoint() {
    maxV = GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel;
    maxAccAtLowV = 6.1;
    maxAccAtHighV = 3.1;
    maxDecelleration = 6.1;
}

} // control
} // ai
} // rtt
