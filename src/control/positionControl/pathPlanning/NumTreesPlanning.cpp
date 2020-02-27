//
// Created by ratoone on 20-02-20.
//

#include "control/positionControl/pathPlanning/NumTreesPlanning.h"

namespace rtt::ai::control{
NumTreesPlanning::NumTreesPlanning(CollisionDetector &collisionDetector) : collisionDetector{collisionDetector}{}

std::vector<Vector2>
NumTreesPlanning::computePath(const Vector2 &robotPosition, const Vector2 &targetPosition) {
    auto root = PathPointNode(robotPosition);
    std::queue<PathPointNode> pointQueue;
    pointQueue.push(root);
    auto finalPath = PathPointNode(targetPosition, root);

    while (!pointQueue.empty()){
        PathPointNode& point = pointQueue.front();
        pointQueue.pop();

        // the algorithm will branch too much if it can't find a proper path; move anyway and try again later
        if (pointQueue.size() >= MAX_BRANCHING){
            break;
        }

        if ((point.getPosition() - targetPosition).length() < TARGET_THRESHOLD){
            finalPath = point;
            break;
        }

        if (point.getParent()){
            auto parentRobotCollision = collisionDetector.getRobotCollisionBetweenPoints(point.getParent()->getPosition(), point.getPosition());
            if (parentRobotCollision){
                auto branches = branchPath(*point.getParent(), parentRobotCollision.value());
                std::for_each(branches.begin(), branches.end(), [&pointQueue](PathPointNode& newPoint){
                    pointQueue.push(newPoint);
                    });
                continue;
            }
        }

        auto robotCollision = collisionDetector.getRobotCollisionBetweenPoints(point.getPosition(), targetPosition);

        // no initial collision
        if (!robotCollision){
            finalPath = PathPointNode(targetPosition, point);
            break;
        }

        auto branches = branchPath(point, robotCollision.value());
        std::for_each(branches.begin(), branches.end(), [&pointQueue](const PathPointNode& newPoint){pointQueue.push(newPoint);});
    }

    std::vector<Vector2> path {finalPath.getPosition()};
    for(PathPointNode& it = finalPath; it.getParent(); it = *it.getParent()){
        path.push_back(it.getParent()->getPosition());
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<PathPointNode> NumTreesPlanning::branchPath(PathPointNode &parentPoint, const Vector2& collisionPosition) const {
    Vector2 deltaPosition = collisionPosition - parentPoint.getPosition();

    Vector2 leftTargetPosition = collisionPosition + Vector2(deltaPosition.y, -deltaPosition.x).stretchToLength(AVOIDANCE_DISTANCE);
    Vector2 rightTargetPosition = collisionPosition + Vector2(deltaPosition.y, -deltaPosition.x).stretchToLength(-AVOIDANCE_DISTANCE);
    return {PathPointNode(leftTargetPosition, parentPoint), PathPointNode(rightTargetPosition, parentPoint)};
}
}