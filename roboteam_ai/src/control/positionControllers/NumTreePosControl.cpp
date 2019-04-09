#include <queue>
#include <roboteam_ai/src/interface/drawer.h>
#include "../../world/Field.h"
#include "NumTreePosControl.h"
#include "PosVelAngle.h"

namespace rtt {
namespace ai {
namespace control {

NumTreePosControl::NumTreePosControl(bool avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        :ForcePosControl(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {
}

/// Clears data and resets variables
void NumTreePosControl::clear() {
    path.clear();
}

/// return the velocity command using two PIDs based on the current position and velocity of the robot compared to the
/// position and velocity of the calculated path
PosVelAngle NumTreePosControl::computeCommand() {
    double goToTimeInFutue = 0.4;
    if (path.size() < static_cast<unsigned int>(1.01 + goToTimeInFutue/DT)) {
        path.clear();
        return {};
    }

    auto pathPoint = path.size() < static_cast<unsigned int>(goToTimeInFutue/DT) ?
                     path.size() : static_cast<unsigned int>(goToTimeInFutue/DT);

    PosVelAngle target;
    target.pos = path[pathPoint].pos;
    target.vel = path[pathPoint].vel;
    target.angle = (target.pos - robot.pos).angle();
    addDataInInterface({{target.pos, Qt::darkMagenta}});

    return target;
}

/// finds a reason to calculate a new path (possible reasons are: on path calculated yet, final target moved,
/// robot is too far from path or another robot is colliding with current path
bool NumTreePosControl::doRecalculatePath(const Vector2 &targetPos) {
    double maxTargetDeviation = 0.3;
    if (path.empty()) {
        if (InterfaceValues::showFullDebugNumTreeInfo())
            std::cout << "no path, recalculating" << std::endl;
        return true;
    }
    else if ((finalTargetPos - targetPos).length() > maxTargetDeviation) {
        if (InterfaceValues::showDebugNumTreeInfo())
            std::cout << "target moved too much, recalculating" << std::endl;
        return true;
    }
    else if (path.size() < static_cast<unsigned int>(1.01 + 0.80/DT)) {
        if ((path[path.size() - 1].pos - targetPos).length() > maxTargetDeviation) {
            if (InterfaceValues::showFullDebugNumTreeInfo())
                std::cout << "reached end of path segment, recalculating" << std::endl;
            return true;
        }
    }

    Vector2 robotPos = robot.pos;
    unsigned long currentIndex = 0;
    double distanceSquared = 9e99;

    for (unsigned long i = 0; i < path.size(); i ++) {
        double pathDistanceToRobot = path[i].pos.dist2(robotPos);
        if (pathDistanceToRobot < distanceSquared) {
            distanceSquared = pathDistanceToRobot;
            currentIndex = i;
        }
    }
    if (sqrt(distanceSquared) > maxTargetDeviation) {
        if (InterfaceValues::showDebugNumTreeInfo())
            std::cout << "robot is too far from current path, recalculating" << std::endl;
        return true;
    }
    std::vector<std::pair<rtt::Vector2, QColor>> colorData = {{}, {}};
    for (int i = 0; i < static_cast<int>(currentIndex); i ++) {
        auto p = path[i];
        colorData.emplace_back(p.pos, Qt::darkGreen);
    }
    path.erase(path.begin(), path.begin() + currentIndex);

    for (auto pathPoint : path) {
        if (getCollision(std::make_shared<PathPoint>(pathPoint), 0.8*DEFAULT_ROBOT_COLLISION_RADIUS).isCollision) {
            if (InterfaceValues::showDebugNumTreeInfo())
                std::cout << "another robot will collide with ours when following this path, recalculating"
                          << std::endl;
            return true;
        }
    }
    addDataInInterface(colorData);
    return false;
}

/// finds a path using a numeric model
PosVelAngle NumTreePosControl::getPosVelAngle(const RobotPtr &robotPtr, Vector2 &targetPos) {
    ros::Time begin = ros::Time::now();

    if (!robotPtr) return calculateForcePosVelAngle(robotPtr, targetPos);
    robot = *robotPtr;

// Check if the current path is still valid, if not, recalculate
    bool nicePath = true;

    PathPointer realRobot = std::make_shared<PathPoint>();
    realRobot->pos = robot.pos;
    realRobot->vel = robot.vel;
    realRobot->t = 0;
    Collision collision = getCollision(realRobot, DEFAULT_ROBOT_COLLISION_RADIUS);
    if (collision.isCollision) {
        const Collision::CollisionType type = collision.getCollisionType();
        std::string s;
        if      (type == Collision::CollisionType::ROBOT)   s = "ROBOT";
        else if (type == Collision::CollisionType::BALL)    s = "BALL";
        else                                                s = "OTHER";

        ros::Time end = ros::Time::now();
        if (InterfaceValues::showDebugNumTreeTimeTaken() && InterfaceValues::showFullDebugNumTreeInfo())
            std::cout << "GoToPosClean tick took: " << (end - begin).toNSec()*0.000001 << " ms" << std::endl;
        if (InterfaceValues::showDebugNumTreeInfo())
            std::cout << "robot " << robot.id << " is too close to " << s << ", trying to use forces instead"
                      << std::endl;

        path.clear();
        return calculateForcePosVelAngle(robotPtr, targetPos);
    }
    if ((targetPos - robot.pos).length() < Constants::MIN_DISTANCE_FOR_FORCE()) {
        path.clear();
        return calculateForcePosVelAngle(robotPtr, targetPos);
    }
    else if (doRecalculatePath(targetPos)) {

        if (Vector2(robot.vel).length() > 10.0) {
            nicePath = false;
            if (InterfaceValues::showDebugNumTreeInfo())
                std::cout << "robot is moving too fast, check world_state?" << std::endl;
        }
        else {
            finalTargetPos = targetPos;

// Calculate new path
            tracePath();

            drawCross(targetPos, Qt::darkRed);
            redrawInInterface();

            if (path.empty())
                nicePath = false;
        }

    }

    ros::Time end = ros::Time::now();
    if (InterfaceValues::showDebugNumTreeTimeTaken() && InterfaceValues::showFullDebugNumTreeInfo())
        std::cout << "GoToPosClean tick took: " << (end - begin).toNSec()*0.000001 << " ms" << std::endl;


    // check if we have a nice path. use forces otherwise.
    if (nicePath) {
        PosVelAngle command = computeCommand();
        return controlWithPID(robotPtr, command);
    }
    else {
        path.clear();
        return calculateForcePosVelAngle(robotPtr, targetPos);
    }
}

void NumTreePosControl::tracePath() {

// compAStar compares the amount of collisions first (max 3 diff), then sorts the paths based on an approximation on the
// length of path that still has to be calculated, using straight lines towards the half-way targets, and the final
// target, while taking into account the length of path already calculated.
    auto compAStar = [this](PathPointer lhs, PathPointer rhs) {
      if (lhs->collisions - rhs->collisions > 2)
          return true;
      else if (lhs->collisions - rhs->collisions < - 2)
          return false;
      else
          return (lhs->maxVel()*lhs->t + remainingStraightLinePathLength(lhs->pos, lhs->currentTarget, finalTargetPos))
                  > (rhs->maxVel()*rhs->t
                          + remainingStraightLinePathLength(rhs->pos, rhs->currentTarget, finalTargetPos));
    };

//// compClose compares the amount of collisions first, then sorts the paths based on an approximation on the length of
//// path that still has to be calculated, using straight lines towards the half-way targets, and then the final target
//    auto compClose = [this](PathPointer lhs, PathPointer rhs) {
//      if (lhs->collisions > rhs->collisions)
//          return true;
//      else if (rhs->collisions > lhs->collisions)
//          return false;
//      else
//          return (remainingStraightLinePathLength(lhs->pos, lhs->currentTarget, finalTargetPos)) >
//                  (remainingStraightLinePathLength(rhs->pos, rhs->currentTarget, finalTargetPos));
//    };

    displayData.clear();
    path.clear();

    PathPointer root = std::make_shared<PathPoint>();
    root->currentTarget = finalTargetPos;
    root->pos = robot.pos;
    root->vel = robot.vel;
    root->acc = {0, 0}; //Assumed for now but could be known from world state/previous commands
    root->t = 0;
    root->collisions = 0;

    std::priority_queue<PathPointer,
                        std::vector<PathPointer>, decltype(compAStar)> pathQueue(compAStar);
    //create root of tree:
    pathQueue.push(root);

    ros::Time start = ros::Time::now();
    while (! pathQueue.empty()) {
        ros::Time now = ros::Time::now();
        if ((now - start).toSec()*1000 > MAX_CALCULATION_TIME) {
            if (InterfaceValues::showDebugNumTreeInfo())
                std::cout << "Tick took too long!" << std::endl;
            //( dont clear path?? ) path.clear();
            return;
        }
        PathPointer point = pathQueue.top();
        while (true) {
            PathPointer newPoint = computeNewPoint(point, point->currentTarget);
            if (newPoint->t > 3.0) {
                path = backTrackPath(point, root);
                return;
            }
            //point->addChild(newPoint);
            point.swap(newPoint);

            // if we reach endpoint, return the path we found
            if (point->isCollision(finalTargetPos, 0.1)) {
                path = backTrackPath(point, root);
                return;
            }
                // if we reach a halfway point, update the target to the final target again and push this new point to queue
            else if (point->isCollision(point->currentTarget, 0.1)) {
                point->currentTarget = finalTargetPos;
                pathQueue.pop();
                pathQueue.push(point);
                break; // break from current while loop, we start looking at different branches again
            }
            // if we have collided with an obstacle; backtrack and branch to both sides using new intermediary points
            Collision collision = getCollision(point, DEFAULT_ROBOT_COLLISION_RADIUS);
            if (collision.isCollision) {
                int collisions = ++ point->collisions;
                std::pair<std::vector<Vector2>, PathPointer> newTargetsAndBranch = getNewTargets(point, collision);
                std::vector<Vector2> newTargets = newTargetsAndBranch.first;
                PathPointer newBranchStart = newTargetsAndBranch.second;

                pathQueue.pop();
                // both left and right targets for now
                for (const auto &newTarget : newTargets) {
                    if (newBranchStart->branchHasTarget(newTarget))
                        continue;
                    // compute new point and add it to branch
                    PathPointer branch = computeNewPoint(newBranchStart, newTarget);
                    branch->collisions = collisions;
                    newBranchStart->addChild(branch);
                    pathQueue.push(branch);
                }
                break; // break from current while loop, we start looking at different branches again
            }
        }

    }
    if (InterfaceValues::showDebugNumTreeInfo())
        std::cout << "reached end of while loop, no path found" << std::endl;
    path = {};
}

/// calculate the remaining pathlength using straigth lines from current position to a posititon halfway and from
/// halfway to the final position
double NumTreePosControl::remainingStraightLinePathLength(
        const Vector2 &currentPos, const Vector2 &halfwayPos, const Vector2 &finalPos) {
    return (abs((halfwayPos - finalPos).length() + (currentPos - halfwayPos).length()));
}

/// create a new pathPoint using a linear acceleration ODE
NumTreePosControl::PathPointer NumTreePosControl::computeNewPoint(
        const PathPointer &oldPoint, const Vector2 &subTarget) {

//Copy parent
    PathPointer newPoint = std::make_shared<PathPoint>();
    newPoint->parent = oldPoint;
    oldPoint->addChild(newPoint);
    newPoint->t = oldPoint->t + DT;
    newPoint->currentTarget = subTarget;
    newPoint->finalTarget = finalTargetPos;
    newPoint->pos = oldPoint->pos;
    newPoint->vel = oldPoint->vel;

    auto dir = (subTarget - newPoint->pos).normalize();
    auto targetVel = dir*newPoint->maxVel();

// change acceleration towards the target velocity
    newPoint->acc = (targetVel - newPoint->vel).stretchToLength(newPoint->maxAcc());

// if the current velocity is away (>90 degrees) from the target, accelerate more towards target
    Angle dAngle = newPoint->vel.toAngle() - dir.toAngle();
    if (fabs(dAngle) > M_PI_2)
        newPoint->acc = (newPoint->acc.normalize() - newPoint->vel.normalize())*newPoint->maxAcc();

//ODE model to get new position/velocity:
    newPoint->vel += newPoint->acc*DT;
    newPoint->pos += newPoint->vel*DT;

    newPoint->collisions = oldPoint->collisions;
    drawPoint(newPoint->pos);

    return newPoint;
}

Collision NumTreePosControl::getCollision(const PathPointer &point, double collisionRadius) {
    // check collisions with robots, the ball, out of field, defense area (and more if needed)
    Collision collision;
    double futureTime = point->t;

    // get all robots and extrapolate their position linearly to the time of the PathPoint (future Robot)
    auto allRobots = world::world->getAllRobots();
    for (auto &r : allRobots) {
        r = world::world->getFutureRobot(r, futureTime);
    }

    // check collision with Robots
    collision = getRobotCollision(point->pos, allRobots, collisionRadius);
    if (collision.isCollision) return collision;

    // get the future Ball
    auto ball = world::world->getFutureBall(futureTime);

    // check collision with Ball
    if (point->isCollision(ball->pos, avoidBallDistance)) {
        collision.setCollisionBall(*ball, avoidBallDistance);
        return collision;
    }

    // check collision with Edge of field
    if (! canMoveOutOfField) {
        if (! world::field->pointIsInField(point->pos)) {
            collision.isCollision = true;
            collision.setOtherCollision(point->pos, 0.2);
            return collision;
        }
    }

    // check collision with defense area
    if (! canMoveInDefenseArea) {
        // our defense area
        auto ourDefenseArea = world::field->getDefenseArea(true, Constants::ROBOT_RADIUS());
        bool isInOurDefenseArea = control::ControlUtils::pointInRectangle(point->pos, ourDefenseArea);

        // their defense area
        auto theirDefenseArea = world::field->getDefenseArea(false, Constants::ROBOT_RADIUS());
        bool isInTheirDefenseArea = control::ControlUtils::pointInRectangle(point->pos, theirDefenseArea);

        if (isInOurDefenseArea || isInTheirDefenseArea) {
            collision.setOtherCollision(point->pos, 0.2);
            return collision;
        }
    }

    return collision;
}

Collision NumTreePosControl::getRobotCollision(
        const Vector2 &collisionPos, const std::vector<Robot> &robots, double distance) {

    // for all robots check if the distance to collisionPos is smaller than the set distance
    Collision collision = {};
    for (auto &r : robots) {
        if (r.id == this->robot.id && r.team == this->robot.team) continue;

        if ((collisionPos - r.pos).length() < distance) {
            collision.setCollisionRobot(r, distance);
            return collision;
        }
    }
    return collision;
}

/// after a collision, get new half-way targets to try to go towards
std::pair<std::vector<Vector2>, NumTreePosControl::PathPointer> NumTreePosControl::getNewTargets(
        const PathPointer &collisionPoint, const Collision &collision) {

    // backtrack to 0.75 seconds before the collision
    double timeBeforeCollision = 0.75;
    PathPointer startPoint = collisionPoint->backTrack(collisionPoint->t - timeBeforeCollision);

    // determine the position at backtrack and vector towards the collision
    Vector2 deltaPosition = collision.collisionPosition() - collisionPoint->pos;
    double collisionRadius = collision.collisionRadius;
    int factor = collisionPoint->collisions - startPoint->collisions;

    // get positions left and right, perpendicular to the vector towards collision, next to the collisionPoint
    Vector2 leftTargetPosition = collisionPoint->pos +
            Vector2(deltaPosition.y, - deltaPosition.x).stretchToLength(collisionRadius*sqrt(factor)*1.2);
    drawCross(leftTargetPosition, Qt::blue);

    Vector2 rightTargetPosition = collisionPoint->pos +
            Vector2(- deltaPosition.y, deltaPosition.x).stretchToLength(collisionRadius*sqrt(factor)*1.2);
    drawCross(rightTargetPosition, Qt::darkBlue);

    // return the new targets
    auto newTargets = {leftTargetPosition, rightTargetPosition};
    return {newTargets, startPoint};
}

///backTracks the path from endPoint until it hits root and outputs in order from root->endPoint
std::vector<PathPoint> NumTreePosControl::backTrackPath(PathPointer point,
        const PathPointer &root) {

    // backtrack the whole path till it hits the root node and return the vector of PathPoints
    std::vector<PathPoint> backTrackedPath = {};
    while (point) {
        backTrackedPath.push_back(*point);
        if (point == root) {
            break;
        }
        point.swap(point->parent);
    }

    std::reverse(backTrackedPath.begin(), backTrackedPath.end()); // everything is from back to forth
    return backTrackedPath;
}

/// draw all the data in the interface
void NumTreePosControl::redrawInInterface() {
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData = {{{}, {}}};
    for (auto &displayAll : displayData) {
        displayColorData.push_back(displayAll);
    }
    for (auto &displayPath : path) {
        displayColorData.emplace_back(displayPath.pos, Qt::red);
    }
    interface::Drawer::setNumTreePoints(robot.id, displayColorData);
}

/// add data to interface
void NumTreePosControl::addDataInInterface(std::vector<std::pair<rtt::Vector2, QColor>> displayColorData) {
    interface::Drawer::addNumTreePoints(robot.id, std::move(displayColorData));
}

/// draw a cross in the interface
void NumTreePosControl::drawCross(Vector2 &pos, const QColor &color) {
// draws a cross for the display
    float dist = 0.005f;
    for (int i = - 14; i < 15; i ++) {
        for (int j = - 1; j < 2; j += 2) {
            Vector2 data = pos + (Vector2) {dist*i, dist*j*i};
            drawPoint(data, color);
        }
    }
}

/// draw a point in the interface
void NumTreePosControl::drawPoint(Vector2 &pos, QColor color) {
    displayData.emplace_back(pos, color);
}

void NumTreePosControl::checkInterfacePID() {
    auto newPid = interface::InterfaceValues::getNumTreePid();
    updatePid(newPid);
}

}// control
}// ai
}// rtt