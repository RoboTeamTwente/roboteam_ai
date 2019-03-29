#include <queue>
#include <roboteam_ai/src/interface/drawer.h>
#include "../../utilities/Field.h"
#include "NumTreePosControl.h"
#include "PosVelAngle.h"

namespace rtt {
namespace ai {
namespace control {

/// Clears data and resets variables
void NumTreePosControl::clear() {
    path.clear();
}

/// Make the Robot able to go outside of the field
void NumTreePosControl::setCanGoOutsideField(bool _canGoOutsideField) {
    canGoOutsideField = _canGoOutsideField;
}

/// return the velocity command using two PIDs based on the current position and velocity of the robot compared to the
/// position and velocity of the calculated path
PosVelAngle NumTreePosControl::computeCommand(RobotPtr robot) {
    if (path.size() < static_cast<unsigned int>(1.01 + 0.30/dt)) {
        path.clear();
        return {};
    }

    auto pathPoint = path.size() < static_cast<unsigned int>(0.60/dt) ?
            path.size() : static_cast<unsigned int>(0.60/dt);

    PosVelAngle target;
    target.pos = path[pathPoint].pos;
    target.vel = path[pathPoint].vel;
    target.angle = (target.pos - robot->pos).angle();
    addDataInInterface({{target.pos, Qt::darkMagenta}});

    return target;
}

/// finds a reason to calculate a new path (possible reasons are: on path calculated yet, final target moved,
/// robot is too far from path or another robot is colliding with current path
bool NumTreePosControl::doRecalculatePath(RobotPtr robot, Vector2 targetPos) {
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
    else if (path.size() < static_cast<unsigned int>(1.01 + 0.80/dt)) {
        if ((path[path.size()-1].pos - targetPos).length() > maxTargetDeviation) {
            if (InterfaceValues::showFullDebugNumTreeInfo())
                std::cout << "reached end of path segment, recalculating" << std::endl;
            return true;
        }
    }

    Vector2 robotPos = robot->pos;
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
    std::vector<std::pair<rtt::Vector2, QColor>> colorData = {{},{}};
    for (int i = 0; i < static_cast<int>(currentIndex); i++) {
        auto p = path[i];
        colorData.emplace_back(p.pos , Qt::darkGreen);
    }
    path.erase(path.begin(), path.begin() + currentIndex);

    for (auto pathPoint : path) {
        if (checkCollision(std::make_shared<PathPoint>(pathPoint), 0.8*defaultRobotCollisionRadius)) {
            if (InterfaceValues::showDebugNumTreeInfo())
                std::cout << "another robot will collide with ours when following this path, recalculating" << std::endl;
            return true;
        }
    }
    addDataInInterface(colorData);
    return false;
}

/// finds a path using a numeric model
PosVelAngle NumTreePosControl::goToPos(RobotPtr robot, Vector2 targetPos) {
    ros::Time begin = ros::Time::now();
    robotID = robot->id;

// Check if the current path is still valid, if not, recalculate
    bool nicePath = true;

    std::shared_ptr<PathPoint> realRobot = std::make_shared<PathPoint>();
    realRobot->pos = robot->pos;
    realRobot->vel = robot->vel;
    realRobot->t = 0;
    if (checkCollision(realRobot, 2.5*Constants::ROBOT_RADIUS_MAX())) {
        ros::Time end = ros::Time::now();
        if (InterfaceValues::showDebugNumTreeTimeTaken() && InterfaceValues::showFullDebugNumTreeInfo())
            std::cout << "GoToPosClean tick took: " << (end-begin).toNSec()*0.000001 << " ms" << std::endl;
        if (InterfaceValues::showDebugNumTreeInfo())
            std::cout << "robot is too close to another robot, trying to use forces instead" << std::endl;

        path.clear();
        return calculateForcePosVelAngle(robot, targetPos);
    }
    if ((targetPos-robot->pos).length() < Constants::MIN_DISTANCE_FOR_FORCE()) {
        path.clear();
        return calculateForcePosVelAngle(robot, targetPos);
    }
    else if (doRecalculatePath(robot, targetPos)) {

        if (Vector2(robot->vel).length() > 10.0) {
            nicePath = false;
            if (InterfaceValues::showDebugNumTreeInfo())
                std::cout << "robot is moving too fast, check world_state?" << std::endl;
        }
        else {
            finalTargetPos = targetPos;

// Calculate new path
            tracePath(robot);

            drawCross(targetPos, Qt::darkRed);
            redrawInInterface();

            if (path.empty())
                nicePath = false;
        }

    }

    ros::Time end = ros::Time::now();
    if (InterfaceValues::showDebugNumTreeTimeTaken() && InterfaceValues::showFullDebugNumTreeInfo())
        std::cout << "GoToPosClean tick took: " << (end-begin).toNSec()*0.000001 << " ms" << std::endl;


    // check if we have a nice path. use forces otherwise.
    if (nicePath) {
        return controlWithPID(robot, computeCommand(robot));
    }
    else {
        path.clear();
        return calculateForcePosVelAngle(robot, targetPos);
    }
}

void NumTreePosControl::tracePath(RobotPtr robot) {

// compAStar compares the amount of collisions first (max 3 diff), then sorts the paths based on an approximation on the
// length of path that still has to be calculated, using straight lines towards the half-way targets, and the final
// target, while taking into account the length of path already calculated.
    auto compAStar = [this](std::shared_ptr<PathPoint> lhs, std::shared_ptr<PathPoint> rhs) {
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
//    auto compClose = [this](std::shared_ptr<PathPoint> lhs, std::shared_ptr<PathPoint> rhs) {
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

    std::shared_ptr<PathPoint> root = std::make_shared<PathPoint>();
    root->currentTarget = finalTargetPos;
    root->pos = robot->pos;
    root->vel = robot->vel;
    root->acc = {0, 0}; //Assumed for now but could be known from world state/previous commands
    root->t = 0;
    root->collisions = 0;

    std::priority_queue<std::shared_ptr<PathPoint>,
                        std::vector<std::shared_ptr<PathPoint>>, decltype(compAStar)> pathQueue(compAStar);
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
        std::shared_ptr<PathPoint> point = pathQueue.top();
        while (true) {
            std::shared_ptr<PathPoint> newPoint = computeNewPoint(point, point->currentTarget);
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
            if (checkCollision(point)) {
                int collisions = ++ point->collisions;
                std::pair<std::vector<Vector2>, std::shared_ptr<PathPoint>> newTargetsAndBranch = getNewTargets(point);
                std::vector<Vector2> newTargets = newTargetsAndBranch.first;
                std::shared_ptr<PathPoint> newBranchStart = newTargetsAndBranch.second;

                pathQueue.pop();
                // both left and right targets for now
                for (const auto &newTarget : newTargets) {
                    if (newBranchStart->branchHasTarget(newTarget))
                        continue;
                    // compute new point and add it to branch
                    std::shared_ptr<PathPoint> branch = computeNewPoint(newBranchStart, newTarget);
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
double NumTreePosControl::remainingStraightLinePathLength(Vector2 currentPos, Vector2 halfwayPos, Vector2 finalPos) {
    return (abs((halfwayPos - finalPos).length() + (currentPos - halfwayPos).length()));
}

/// create a new pathPoint using a linear acceleration ODE
std::shared_ptr<PathPoint> NumTreePosControl::computeNewPoint(
        std::shared_ptr<PathPoint> oldPoint, Vector2 subTarget) {

//Copy parent
    std::shared_ptr<PathPoint> newPoint = std::make_shared<PathPoint>();
    newPoint->parent = oldPoint;
    oldPoint->addChild(newPoint);
    newPoint->t = oldPoint->t + dt;
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
    newPoint->vel += newPoint->acc*dt;
    newPoint->pos += newPoint->vel*dt;

    newPoint->collisions = oldPoint->collisions;
    drawPoint(newPoint->pos);

    return newPoint;
}

/// check if a pathpoint is in a collision with a robot/ball at that timepoint
bool NumTreePosControl::checkCollision(std::shared_ptr<PathPoint> point, double collisionRadius) {
    auto w = world::world->getWorld();
    for (auto bot : w.us) {
        if (bot.id != static_cast<int>(robotID)) {
            Vector2 botPos = (Vector2) (bot.pos) + (Vector2) (bot.vel)*point->t;
            if (point->isCollision(botPos, collisionRadius))
                return true;
        }
    }
    for (auto bot: w.them) {
        Vector2 botPos = (Vector2) (bot.pos) + (Vector2) (bot.vel)*point->t;
        if (point->isCollision(botPos, collisionRadius))
            return true;
    }
    if (avoidBall) {
        Vector2 ballPos = (Vector2) (w.ball.pos) + (Vector2) (w.ball.vel)*point->t;
        if (point->isCollision(ballPos, collisionRadius*0.5 + Constants::BALL_RADIUS()))
            return true;
    }
    if (!canGoOutsideField) {
        if (world::field->pointIsInField(point->pos))
            return true;
    }
    return false;
}

/// find the robot corresponding to a collision-position
Vector2 NumTreePosControl::findCollisionPos(std::shared_ptr<PathPoint> point, double collisionRadius) {
    auto w = world::world->getWorld();
    for (auto bot: w.us) {
        if (bot.id != robotID) {
            Vector2 botPos = (Vector2) (bot.pos) + (Vector2) (bot.vel)*point->t;
            if (point->isCollision(botPos, collisionRadius)) {
                return botPos;
            }
        }
    }
    for (auto bot: w.them) {
        Vector2 botPos = (Vector2) (bot.pos) + (Vector2) (bot.vel)*point->t;
        if (point->isCollision(botPos, collisionRadius)) {
            return botPos;
        }
    }
    if (avoidBall) {
        Vector2 ballPos = (Vector2) (w.ball.pos) + (Vector2) (w.ball.vel)*point->t;
        if (point->isCollision(ballPos, collisionRadius*0.5 + Constants::BALL_RADIUS())) {
            return ballPos;
        }
    }
    if (!canGoOutsideField) {
        if (world::field->pointIsInField(point->pos))
            return point->pos;
    }
    return {- 42, 42};

}

/// after a collision, get new half-way targets to try to go towards
std::pair<std::vector<Vector2>, std::shared_ptr<PathPoint>> NumTreePosControl::getNewTargets(
        std::shared_ptr<PathPoint> collisionPoint) {

    std::shared_ptr<PathPoint> initialBranchStart = collisionPoint->backTrack(collisionPoint->t - 0.91);
    int factor = collisionPoint->collisions - initialBranchStart->collisions;
    std::shared_ptr<PathPoint> newBranchStart = initialBranchStart;//->backTrack(initialBranchStart->t - 0.1*factor);

    Vector2 collisionPosition = findCollisionPos(collisionPoint);
    Vector2 startPosition = newBranchStart->pos;
    Vector2 deltaPosition = collisionPosition - startPosition;

    double length = deltaPosition.length();
    double defaultDistanceFromCollision = 0.3;
    double deltaAngle = sqrt(static_cast<double>(factor))*atan(defaultDistanceFromCollision/length);

    Vector2 leftPosition = startPosition + deltaPosition.rotate(deltaAngle);
    drawCross(leftPosition, Qt::blue);

    Vector2 rightPosition = startPosition + deltaPosition.rotate(- deltaAngle);
    drawCross(rightPosition, Qt::darkBlue);

    std::vector<Vector2> newTargets = {leftPosition, rightPosition};

    return {newTargets, newBranchStart};
}



///backTracks the path from endPoint until it hits root and outputs in order from root->endPoint
std::vector<PathPoint> NumTreePosControl::backTrackPath(std::shared_ptr<PathPoint> point,
        std::shared_ptr<PathPoint> root) {

    std::vector<PathPoint> path = {};
    while (point) {
        path.push_back(*point);
        if (point == root) {
            break;
        }
        point.swap(point->parent);
    }

    std::reverse(path.begin(), path.end()); // everything is from back to forth
    return path;
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
    interface::Drawer::setNumTreePoints(robotID, displayColorData);
}

/// add data to interface
void NumTreePosControl::addDataInInterface(std::vector<std::pair<rtt::Vector2, QColor>> displayColorData) {
    interface::Drawer::addNumTreePoints(robotID, std::move(displayColorData));
}

/// draw a cross in the interface
void NumTreePosControl::drawCross(Vector2 &pos, QColor color) {
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

}// control
}// ai
}// rtt