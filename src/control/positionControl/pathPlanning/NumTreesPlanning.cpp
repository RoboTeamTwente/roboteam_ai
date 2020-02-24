//
// Created by ratoone on 20-02-20.
//

#include <include/roboteam_ai/control/positionControl/PathPointNode.h>
#include <queue>
#include "control/positionControl/pathPlanning/NumTreesPlanning.h"

namespace rtt::ai::control{
NumTreesPlanning::NumTreesPlanning(CollisionDetector &collisionDetector) : collisionDetector{collisionDetector}{}

std::vector<Vector2>
NumTreesPlanning::computePath(const Vector2 &robotPosition, const Vector2 &targetPosition) {
    auto root = PathPointNode(robotPosition);
    std::queue<PathPointNode> pointQueue;
    pointQueue.push(root);
    auto finalPath = PathPointNode(targetPosition);

    while (!pointQueue.empty()){
        PathPointNode& point = pointQueue.front();
        pointQueue.pop();
        if ((point.getPosition() - targetPosition).length() < TARGET_THRESHOLD){
            finalPath = point;
            break;
        }

        //TODO: add parent collision check
//        auto parentRobotCollision = collisionDetector.getRobotCollisionBetweenPoints(point.getParent()->getPosition(), point.getPosition());
        auto robotCollision = collisionDetector.getRobotCollisionBetweenPoints(point.getPosition(), targetPosition);

        // no initial collision
        if (!robotCollision){
            finalPath = PathPointNode(targetPosition);
            finalPath.setParent(point);
            break;
        }

        Vector2 deltaPosition = robotCollision.value() - point.getPosition();

        Vector2 leftTargetPosition = robotCollision.value() + Vector2(deltaPosition.y, -deltaPosition.x).stretchToLength(AVOIDANCE_DISTANCE);
        auto leftNode = PathPointNode(leftTargetPosition);
        leftNode.setParent(point);

        Vector2 rightTargetPosition = robotCollision.value() + Vector2(deltaPosition.y, -deltaPosition.x).stretchToLength(-AVOIDANCE_DISTANCE);
        auto rightNode = PathPointNode(rightTargetPosition);
        rightNode.setParent(point);

        pointQueue.push(leftNode);
        pointQueue.push(rightNode);
    }

    std::vector<Vector2> path {finalPath.getPosition()};
    for(PathPointNode& it = finalPath; it.getParent(); it = *it.getParent()){
        path.push_back(it.getParent()->getPosition());
    }
    std::reverse(path.begin(), path.end());
    return path;
}
}