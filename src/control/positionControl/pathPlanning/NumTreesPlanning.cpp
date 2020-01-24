//
// Created by ratoone on 09-12-19.
//

#include "control/positionControl/pathPlanning/NumTreesPlanning.h"

namespace rtt::ai::control{

NumTreesPlanning::NumTreesPlanning(const CollisionDetector &collisionDetector) :
    collisionDetector(collisionDetector) {}

std::list<rtt::Vector2>
NumTreesPlanning::computePath(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition) {
    DT = 0.3 / rtt::ai::GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel;
    if (DT > 0.12) DT = 0.12;
    if (DT < 0.06) DT = 0.06;

    auto path = tracePath(robotPosition, targetPosition);
    double goToTimeInFuture = 0.4;
    auto targetPathPoint = static_cast<unsigned long>(goToTimeInFuture/DT);
    if (path.size() < targetPathPoint) {
        return std::list<Vector2>(1, targetPosition);
    }
    return path;
}

std::list<Vector2> NumTreesPlanning::tracePath(const rtt::Vector2 &currentPosition, const rtt::Vector2 &targetPosition) {

// compAStar compares the amount of collisions first (max 3 diff), then sorts the paths based on an approximation on the
// length of path that still has to be calculated, using straight lines towards the half-way targets, and the final
// target, while taking into account the length of path already calculated.
    auto compAStar = [targetPosition, this](PathPointer lhs, PathPointer rhs) {
        if (lhs->collisions - rhs->collisions > 2)
            return true;
        else if (lhs->collisions - rhs->collisions < - 2)
            return false;
        else
            return (lhs->maxVel()*lhs->t + remainingStraightLinePathLength(lhs->pos, lhs->currentTarget, targetPosition))
                   > (rhs->maxVel()*rhs->t
                      + remainingStraightLinePathLength(rhs->pos, rhs->currentTarget, targetPosition));
    };

    std::vector<rtt::ai::control::PathPoint> path;

    // create a new path with a root
    std::priority_queue<PathPointer, std::vector<PathPointer>, decltype(compAStar)> pathQueue(compAStar);
    PathPointer root = std::make_shared<rtt::ai::control::PathPoint>();
    root->currentTarget = targetPosition;
    root->pos = currentPosition;
    root->vel = Vector2();
    root->acc = Vector2();
    root->t = 0;
    root->collisions = 0;
    pathQueue.push(root);

    auto start = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );
    while (! pathQueue.empty()) {
        auto now = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()
        );
        if ((now - start).count()*1000 > MAX_CALCULATION_TIME) {
            if (rtt::ai::interface::Output::showDebugNumTreeInfo()) {
                std::cout << "ROBOT " << ": Tick took too long!" << std::endl;
            }
            PathPointer bestPath = pathQueue.top();
            pathQueue.pop();
            while (! pathQueue.empty()) {
                PathPointer pathPointer = pathQueue.top();
                if ((targetPosition - pathPointer->pos).length2() < (targetPosition - bestPath->pos).length2()) {
                    bestPath = pathPointer;
                }
                pathQueue.pop();
            }
            return backTrackPath(bestPath, root);
        }

        PathPointer point = pathQueue.top();
        while (true) {
            PathPointer newPoint = computeNewPoint(targetPosition, point, point->currentTarget);
            if (newPoint->t > 3.0) {
                return backTrackPath(point, root);
            }
            //point->addChild(newPoint);
            point.swap(newPoint);

            // if we reach endpoint, return the path we found
            if (point->isCollision(targetPosition, 0.1)) {
                return backTrackPath(point, root);
            }
                // if we reach a halfway point, update the target to the final target again and push this new point to queue
            else if (point->isCollision(point->currentTarget, 0.1)) {
                point->currentTarget = targetPosition;
                pathQueue.pop();
                pathQueue.push(point);
                break; // break from current while loop, we start looking at different branches again
            }
            // if we have collided with an obstacle; backtrack and branch to both sides using new intermediary points
            if (collisionDetector.isCollisionBetweenPoints(point->parent->pos, point->pos)) {
                int collisions = ++ point->collisions;
                std::pair<std::vector<Vector2>, PathPointer> newTargetsAndBranch = getNewTargets(point);
                std::vector<Vector2> newTargets = newTargetsAndBranch.first;
                PathPointer newBranchStart = newTargetsAndBranch.second;

                pathQueue.pop();
                // both left and right targets for now
                for (const auto &newTarget : newTargets) {
                    if (newBranchStart->branchHasTarget(newTarget))
                        continue;
                    // compute new point and add it to branch
                    PathPointer branch = computeNewPoint(targetPosition, newBranchStart, newTarget);
                    branch->collisions = collisions;
                    newBranchStart->addChild(branch);
                    pathQueue.push(branch);
                }
                break; // break from current while loop, we start looking at different branches again
            }
        }
    }
    if (rtt::ai::interface::Output::showDebugNumTreeInfo()) {
        std::cout << "ROBOT " << ": reached end of while loop, no path found" << std::endl;
    }
    return {};
}

/// calculate the remaining pathlength using straigth lines from current position to a posititon halfway and from
/// halfway to the final position
double NumTreesPlanning::remainingStraightLinePathLength(
        const Vector2 &currentPos, const Vector2 &halfwayPos, const Vector2 &finalPos) {
    return (abs((halfwayPos - finalPos).length() + (currentPos - halfwayPos).length()));
}

///backTracks the path from endPoint until it hits root and outputs in order from root->endPoint
std::list<Vector2> NumTreesPlanning::backTrackPath(PathPointer point,
                                                        const PathPointer &root) {

    // backtrack the whole path till it hits the root node and return the vector of PathPoints
    std::list<Vector2> backTrackedPath = {};
    while (point) {
        backTrackedPath.push_front(point->pos);
        if (point == root) {
            break;
        }
        point.swap(point->parent);
    }

    return backTrackedPath;
}

/// create a new pathPoint using a linear acceleration ODE
NumTreesPlanning::PathPointer NumTreesPlanning::computeNewPoint(const rtt::Vector2 &targetPosition,
        const PathPointer &oldPoint, const rtt::Vector2 &subTarget) {

    // Copy parent
    PathPointer newPoint = std::make_shared<rtt::ai::control::PathPoint>();
    newPoint->parent = oldPoint;
    oldPoint->addChild(newPoint);
    newPoint->t = oldPoint->t + DT;
    newPoint->currentTarget = subTarget;
    newPoint->finalTarget = targetPosition;
    newPoint->pos = oldPoint->pos;
    newPoint->vel = oldPoint->vel;

    // accelerate towards the target
    // if the current velocity is away (>90 degrees) from the target, accelerate more towards target
    Vector2 targetDirection = (subTarget - newPoint->pos);
    Angle deltaAngle = newPoint->vel.toAngle() - targetDirection.toAngle();

    newPoint->acc = ((targetDirection.normalize()*newPoint->maxVel() - newPoint->vel).normalize() -
                     newPoint->vel.normalize()*fabs(deltaAngle)*M_2_PI);

    Vector2 targetVel = newPoint->vel + newPoint->acc.stretchToLength(DT*
                                                                      std::max(newPoint->maxAcceleration(), newPoint->maxDeceleration()));
    Angle targetAngle = (targetVel - oldPoint->vel).toAngle();

    //ODE model to get new position/velocity:
    newPoint->vel = ControlUtils::accelerationLimiter(targetVel, oldPoint->vel, targetAngle,
                                                      newPoint->maxAcceleration()*DT, newPoint->maxAcceleration()*DT,
                                                      newPoint->maxDeceleration()*DT, newPoint->maxDeceleration()*DT);
    newPoint->pos += newPoint->vel*DT;

    newPoint->collisions = oldPoint->collisions;
    return newPoint;
}

/// after a collision, get new half-way targets to try to go towards
    std::pair<std::vector<Vector2>, NumTreesPlanning::PathPointer>
    NumTreesPlanning::getNewTargets(const NumTreesPlanning::PathPointer &collisionPoint) {

    // backtrack to 0.75 seconds before the collision
    double timeBeforeCollision = 0.75;
    PathPointer startPoint = collisionPoint->backTrack(collisionPoint->t - timeBeforeCollision);

    // determine the position at backtrack and vector towards the collision
    //TODO: workaround warning: used half the distance instead of collision distance
    Vector2 collisionPosition = (collisionPoint->parent->pos + collisionPoint->pos)/2;
    Vector2 deltaPosition = collisionPosition - collisionPoint->pos;
    double collisionRadius = DEFAULT_ROBOT_COLLISION_RADIUS;
    int factor = collisionPoint->collisions - startPoint->collisions;

    // get positions left and right, perpendicular to the vector towards collision, next to the collisionPoint
    Vector2 leftTargetPosition = collisionPoint->pos +
                                 Vector2(deltaPosition.y, - deltaPosition.x).stretchToLength(collisionRadius*sqrt(factor)*1.2);

    Vector2 rightTargetPosition = collisionPoint->pos +
                                  Vector2(deltaPosition.y, - deltaPosition.x).stretchToLength(- collisionRadius*sqrt(factor)*1.2);

    // return the new targets
    auto newTargets = {leftTargetPosition, rightTargetPosition};
    return {newTargets, startPoint};
}

}