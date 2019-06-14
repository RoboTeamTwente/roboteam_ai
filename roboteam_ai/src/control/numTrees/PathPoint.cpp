//
// Created by mrlukasbos on 28-3-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "PathPoint.h"

namespace rtt {
namespace ai {
namespace control {

/// go back in the path until desired time or until Root
const PathPoint::PathPointer &PathPoint::backTrack(const PathPointer &point, double backTime) {
    if (! point->parent)
        return point;
    else if (backTime > point->t)
        return point;
    else
        return backTrack(point->parent, backTime);
}

/// go back in the path until desired collision difference or until Root
const PathPoint::PathPointer &PathPoint::backTrack(const PathPointer &point, int maxCollisionDiff) {
    if (! point->parent)
        return point;
    else if (maxCollisionDiff == 0)
        return point;
    else
        return backTrack(point->parent, point->collisions - point->parent->collisions);
}

/// go back in the path until desired time, collision difference or until Root
const PathPoint::PathPointer &PathPoint::backTrack(const PathPointer &point, double backTime, int maxCollisionDiff) {

    if (! point->parent)
        return point;
    else if (maxCollisionDiff == 0)
        return point;
    else if (point->collisions > point->parent->collisions)
        return backTrack(point->parent, maxCollisionDiff - 1);
    else if (backTime > point->t)
        return point;
    else
        return backTrack(point->parent, backTime, maxCollisionDiff);
}


///backTracks the path from endPoint until it hits root and outputs in order from root->endPoint
const std::vector<PathPoint> PathPoint::backTrackPath(PathPointer start, const PathPointer &root) {

    // backtrack the whole path till it hits the root node and return the vector of PathPoints
    std::vector<PathPoint> backTrackedPath = {};
    PathPointer &point = start;
    while (point) {
        backTrackedPath.push_back(*point);
        if (point == root) {
            break;
        }
        point = point->parent;
    }

    std::reverse(backTrackedPath.begin(), backTrackedPath.end()); // everything is from back to forth
    return backTrackedPath;
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
    auto root = backTrack(std::make_shared<PathPoint>(*this), 0.0);
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
    double absoluteMax = sqrt(2.0*Constants::MAX_DEC_UPPER()*distanceRemaining)*0.8;
    return absoluteMax > maxVelocity() ? maxVelocity() : absoluteMax;
}

const PathPoint::PathPointer &PathPoint::getParent() const {
    return parent;
}

const std::vector<PathPoint::PathPointer> &PathPoint::getChildren() const {
    return children;
}

void PathPoint::setParent(const PathPoint::PathPointer &p) {
    parent = p;
}

void PathPoint::setChildren(const std::vector<PathPoint::PathPointer> &c) {
    children = c;
}

} // control
} // ai
} // rtt
