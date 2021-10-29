//
// Created by ratoone on 20-02-20.
//

#include "control/positionControl/pathPlanning/NumTreesPlanning.h"

namespace rtt::ai::control {
NumTreesPlanning::NumTreesPlanning(CollisionDetector &collisionDetector) : PathPlanningAlgorithm(collisionDetector) {}

std::vector<Vector2> NumTreesPlanning::computePath(const Vector2 &robotPosition, Vector2 &targetPosition) {
    auto root = PathPointNode(robotPosition);
    std::queue<PathPointNode> pointQueue;
    pointQueue.push(root);
    auto finalPath = root;
    int iterations = 0;

    while (!pointQueue.empty()) {
        PathPointNode &point = pointQueue.front();
        pointQueue.pop();
        iterations++;

        // the algorithm will branch too much if it can't find a proper path; move anyway and try again later
        if (pointQueue.size() >= MAX_BRANCHING || iterations > MAX_ITERATIONS) {
            break;
        }

        if ((point.getPosition() - targetPosition).length() < TARGET_THRESHOLD) {
            finalPath = point;
            break;
        }

        if (point.getParent()) {
            if (!collisionDetector.isPointInsideField(point.getPosition())) {
                continue;
            }
            auto parentCollision = collisionDetector.getCollisionBetweenPoints(point.getParent()->getPosition(), point.getPosition());
            if (parentCollision) {
                auto branches = branchPath(*point.getParent(), parentCollision.value(), targetPosition);
                std::for_each(branches.begin(), branches.end(), [&pointQueue](PathPointNode &newPoint) { pointQueue.push(newPoint); });
                continue;
            }
            // current path has no collisions - update current best path
            if (point.getPosition() - targetPosition < finalPath.getPosition() - targetPosition) {
                finalPath = point;
            }
        }

        auto collision = collisionDetector.getCollisionBetweenPoints(point.getPosition(), targetPosition);

        // no initial collision
        if (!collision) {
            finalPath = PathPointNode(targetPosition, point);
            break;
        }

        auto branches = branchPath(point, collision.value(), targetPosition);
        std::for_each(branches.begin(), branches.end(), [&pointQueue](const PathPointNode &newPoint) { pointQueue.push(newPoint); });
    }

    std::vector<Vector2> path{finalPath.getPosition()};
    for (PathPointNode &it = finalPath; it.getParent(); it = *(it.getParent())) {
        path.push_back(it.getParent()->getPosition());
    }
    std::reverse(path.begin(), path.end());
    // path should contain final point - if there are collisions between the last 2 points, the path will be recalculated
    // closer to the destination
    if (path.back() != targetPosition) {
        path.push_back(targetPosition);
    }
    return path;
}

std::vector<PathPointNode> NumTreesPlanning::branchPath(PathPointNode &parentPoint, const Vector2 &collisionPosition, Vector2 &destination) const {
    // const rtt_world::Field &field, const Vector2 &point, bool isOurDefenceArea, double margin, double backMargin
    if (collisionPosition.x > 1.9 && abs(collisionPosition.y) < 1.1) {
        destination = (destination - Vector2(4, 0)).stretchToLength(0.5) + destination;
    }
    Vector2 deltaPosition = collisionPosition - parentPoint.getPosition();

    Vector2 leftTargetPosition = collisionPosition + Vector2(deltaPosition.y, -deltaPosition.x).stretchToLength(AVOIDANCE_DISTANCE);
    Vector2 rightTargetPosition = collisionPosition + Vector2(deltaPosition.y, -deltaPosition.x).stretchToLength(-AVOIDANCE_DISTANCE);

    // pick the closest branch to try first
    if (leftTargetPosition - destination < rightTargetPosition - destination) {
        return {PathPointNode(leftTargetPosition, parentPoint), PathPointNode(rightTargetPosition, parentPoint)};
    }
    return {PathPointNode(rightTargetPosition, parentPoint), PathPointNode(leftTargetPosition, parentPoint)};
}
}  // namespace rtt::ai::control
