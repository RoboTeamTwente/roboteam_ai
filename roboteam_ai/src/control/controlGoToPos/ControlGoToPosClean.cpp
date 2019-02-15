//
// Created by rolf on 5-2-19.
//

#include "ControlGoToPosClean.h"

namespace rtt {
namespace ai {
namespace control {

/// Turn ball avoidance off and on
void ControlGoToPosClean::setAvoidBall(bool _avoidBall) {
    avoidBall = _avoidBall;
}

/// Make the Robot able to go outside of the field
void ControlGoToPosClean::setCanGoOutsideField(bool _canGoOutsideField) {
    canGoOutsideField = _canGoOutsideField;
}

/// return the velocity command using two PIDs based on the current position and velocity of the robot compared to the
/// position and velocity of the calculated path
Vector2 ControlGoToPosClean::computeNumericCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot) {
    if (path.empty())
        return computeForceCommand(robot);

    int pathPoint = static_cast<int>(path.size() > 4 ? 4 : path.size());

    Vector2 pidVel = path[pathPoint].vel;
    Vector2 pidV = velPID.controlPIR(pidVel, robot->vel);

    Vector2 pidPos = path[pathPoint].pos;
    Vector2 pidP = posPID.controlPID(pidPos - robot->pos);

    return pidV + pidP;
}

/// return the velocity command using a force model, where the robot gets repelled from robots close by,
/// while also adding a velocity vector towards the final target
Vector2 ControlGoToPosClean::computeForceCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot) {
    path.clear();
    roboteam_msgs::World world = World::get_world();
    Vector2 force = (finalTargetPos - robot->pos).normalize();
    for (auto bot : world.us) {
        force = force
                + ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, 1, defaultRobotCollisionRadius*2.0);
    }
    for (auto bot : world.them) {
        force = force
                + ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, 1, defaultRobotCollisionRadius*2.0);
    }
    force = (force.length() > 3.0 ? force.stretchToLength(3.0) : force);
    return force;
}

/// executes numeric or force velocity command based on input
Vector2 ControlGoToPosClean::computeCommand(std::shared_ptr<roboteam_msgs::WorldRobot> robot, GTPType gtpType) {
    switch (gtpType) {
    case numeric:return computeNumericCommand(std::move(robot));
    case force:return computeForceCommand(std::move(robot));
    }
    return computeNumericCommand(std::move(robot));
}

/// finds a reason to calculate a new path (possible reasons are: on path calculated yet, final target moved,
/// robot is too far from path or another robot is colliding with current path
bool ControlGoToPosClean::doRecalculatePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos) {
    double maxTargetDeviation = 0.3;
    if (path.empty()) {
        std::cout << "no path, recalculating" << std::endl;
        return true;
    }
    else if ((finalTargetPos - targetPos).length() > maxTargetDeviation) {
        std::cout << "target moved too much, recalculating" << std::endl;
        return true;
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
        std::cout << "robot is too far from current path, recalculating" << std::endl;
        return true;
    }
    path.erase(path.begin(), path.begin() + currentIndex);

    for (auto pathPoint : path) {
        if (checkCollision(std::make_shared<PathPoint>(pathPoint))) {
            std::cout << "another robot will collide with ours when following this path, recalculating" << std::endl;
            return true;
        }
    }

    return false;
}

/// finds a path using a numeric model
Vector2 ControlGoToPosClean::goToPos(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos) {
    robotID = robot->id;

// init or change PID
    if (! pidInit) {
        pidInit = true;
        initializePID();
    }
    checkInterfacePID();

// check if the current path is still valid, if not, recalculate
    bool nicePath = true;

    std::shared_ptr<PathPoint> realRobot = std::make_shared<PathPoint>();
    realRobot->pos = robot->pos;
    realRobot->vel = robot->vel;
    realRobot->t = 0;
    if (checkCollision(realRobot)) {
        std::cout << "robot is too close to another robot, trying other GoToPos???" << std::endl;
        path.clear();
        return computeCommand(robot, GTPType::force);
    }
    else if (doRecalculatePath(robot, targetPos)) {
        if (Vector2(robot->vel).length() > 10.0) {
            nicePath = false;
        }
        else {
            finalTargetPos = targetPos;

            tracePath(robot);

            if (path.empty())
                nicePath = false;
        }
    }

// draw in the interface
    drawCross(targetPos, Qt::darkRed);
    drawInInterface();

// compute command using PID
    if (nicePath)
        return computeCommand(robot);
    else
        return computeCommand(robot, GTPType::force);
}

void ControlGoToPosClean::tracePath(std::shared_ptr<roboteam_msgs::WorldRobot> robot) {

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
                  >
                          (rhs->maxVel()*rhs->t
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

    displayData = {};
    velPID.reset();
    posPID.reset();
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
        if ((now - start).toSec()*1000 > Constants::MAX_CALCULATION_TIME()) {
            std::cout << "Tick took too long!" << std::endl;
            path = {};
            return;
        }

        std::shared_ptr<PathPoint> point = pathQueue.top();
        while (true) {
            std::shared_ptr<PathPoint> newPoint = computeNewPoint(point, point->currentTarget);
            point->addChild(newPoint);
            point = newPoint;

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
                    if (newBranchStart->anyBranchHasTarget(newTarget))
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
    std::cout << "reached end of while loop: " << std::endl;
    path = {};
}

/// calculate the remaining pathlength using straigth lines from current position to a posititon halfway and from
/// halfway to the final position
double ControlGoToPosClean::remainingStraightLinePathLength(Vector2 currentPos, Vector2 halfwayPos, Vector2 finalPos) {
    return (abs((halfwayPos - finalPos).length() + (currentPos - halfwayPos).length()));
}

/// create a new pathPoint using a linear acceleration ODE
std::shared_ptr<ControlGoToPosClean::PathPoint> ControlGoToPosClean::computeNewPoint(
        std::shared_ptr<PathPoint> oldPoint, Vector2 subTarget) {

    std::shared_ptr<PathPoint> newPoint = std::make_shared<PathPoint>();
    newPoint->parent = oldPoint;
    newPoint->t = oldPoint->t + dt;
    newPoint->currentTarget = subTarget;
    newPoint->finalTarget = finalTargetPos;
    newPoint->hasBeenTicked = true;
    newPoint->pos = oldPoint->pos;
    newPoint->vel = oldPoint->vel;
    newPoint->acc = oldPoint->acc;

    //ODE model:
    Vector2 targetVel = (subTarget - oldPoint->pos).normalize()*newPoint->maxVel();
    double angle = abs((targetVel - oldPoint->vel).angle());
    //angle = angle > M_PI ? M_2_PI - angle : angle;
    newPoint->acc = (targetVel - oldPoint->vel*
            (2.0 + (abs(angle) > M_PI_2 ? abs(angle) - M_PI_2 : 0.0))).stretchToLength(oldPoint->maxAcc());
    newPoint->vel = oldPoint->vel + newPoint->acc*dt;
    newPoint->pos = oldPoint->pos + newPoint->vel*dt;

    newPoint->collisions = oldPoint->collisions;
    drawPoint(newPoint->pos);

    return newPoint;
}

/// check if a pathpoint is in a collision with a robot/ball at that timepoint
bool ControlGoToPosClean::checkCollision(std::shared_ptr<PathPoint> point, double collisionRadius) {
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    roboteam_msgs::World world = World::get_world();
    for (auto bot : world.us) {
        if (bot.id != static_cast<unsigned long>(robotID)) {
            Vector2 botPos = (Vector2) (bot.pos) + (Vector2) (bot.vel)*point->t;
            if (point->isCollision(botPos, collisionRadius)) {
                std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
                time = time + (end - start);
                return true;
            }
        }
    }
    for (auto bot: world.them) {
        Vector2 botPos = (Vector2) (bot.pos) + (Vector2) (bot.vel)*point->t;
        if (point->isCollision(botPos, collisionRadius)) {
            std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
            time = time + (end - start);
            return true;
        }
    }
    if (avoidBall) {
        Vector2 ballPos = (Vector2) (world.ball.pos) + (Vector2) (world.ball.vel)*point->t;
        if (point->isCollision(ballPos, collisionRadius*0.5 + Constants::BALL_RADIUS())) {
            std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
            time = time + (end - start);
            return true;
        }
    }
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    time = time + (end - start);
    return false;
}

/// find the robot corresponding to a collision-position
Vector2 ControlGoToPosClean::findCollisionPos(std::shared_ptr<PathPoint> point, double collisionRadius) {
    roboteam_msgs::World world = World::get_world();
    for (auto bot: world.us) {
        if (bot.id != static_cast<unsigned long>(robotID)) {
            Vector2 botPos = (Vector2) (bot.pos) + (Vector2) (bot.vel)*point->t;
            if (point->isCollision(botPos, collisionRadius)) {
                return botPos;
            }
        }
    }
    for (auto bot: world.them) {
        Vector2 botPos = (Vector2) (bot.pos) + (Vector2) (bot.vel)*point->t;
        if (point->isCollision(botPos, collisionRadius)) {
            return botPos;
        }
    }
    if (avoidBall) {
        Vector2 ballPos = (Vector2) (world.ball.pos) + (Vector2) (world.ball.vel)*point->t;
        if (point->isCollision(ballPos, collisionRadius*0.5 + Constants::BALL_RADIUS())) {
            return ballPos;
        }
    }
    return {- 42, 42};

}

/// after a collision, get new half-way targets to try to go towards
std::pair<std::vector<Vector2>, std::shared_ptr<ControlGoToPosClean::PathPoint>> ControlGoToPosClean::getNewTargets(
        std::shared_ptr<PathPoint> collisionPoint) {

    std::shared_ptr<PathPoint> initialBranchStart = collisionPoint->backTrack(collisionPoint->t - 0.71);
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

/// go back in the path until desired time or until Root
std::shared_ptr<ControlGoToPosClean::PathPoint> ControlGoToPosClean::PathPoint::backTrack(double backTime) {
    if (! parent)
        return shared_from_this();
    else if (backTime > t)
        return shared_from_this();
    else
        return parent->backTrack(backTime);
}

/// go back in the path until desired collision difference or until Root
std::shared_ptr<ControlGoToPosClean::PathPoint> ControlGoToPosClean::PathPoint::backTrack(int maxCollisionDiff) {
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
std::shared_ptr<ControlGoToPosClean::PathPoint> ControlGoToPosClean::PathPoint::backTrack(double backTime,
        int maxCollisionDiff) {

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

///backTracks the path from endPoint until it hits root and outputs in order from root->endPoint
std::vector<ControlGoToPosClean::PathPoint> ControlGoToPosClean::backTrackPath(std::shared_ptr<PathPoint> endPoint,
        std::shared_ptr<PathPoint> root) {
    std::vector<PathPoint> path;
    std::shared_ptr<PathPoint> point = std::move(endPoint);
    while (point) {
        path.push_back(*point);
        if (point == root) {
            break;
        }
        point = point->parent;
    }
    std::reverse(path.begin(), path.end()); // everything is from back to forth
    return path;
}

/// start the PID for velocity and position control
void ControlGoToPosClean::initializePID() {
    velPID.reset();
    velPID.setPID(Constants::standard_luth_P(),
            Constants::standard_luth_P(),
            Constants::standard_luth_P());

    posPID.reset();
    posPID.setPID(Constants::standard_luth_P(),
            Constants::standard_luth_P(),
            Constants::standard_luth_P());
}

/// compare current PID values to those set in the interface
void ControlGoToPosClean::checkInterfacePID() {
    if (velPID.getP() != interface::InterfaceValues::getLuthP() ||
            velPID.getI() != interface::InterfaceValues::getLuthI() ||
            velPID.getD() != interface::InterfaceValues::getLuthD()) {
        velPID.reset();
        velPID.setPID(interface::InterfaceValues::getLuthP(),
                interface::InterfaceValues::getLuthI(),
                interface::InterfaceValues::getLuthD());

        posPID.reset();
        posPID.setPID(interface::InterfaceValues::getLuthP(),
                interface::InterfaceValues::getLuthI(),
                interface::InterfaceValues::getLuthD());
    }
}

/// draw all the data in the interface
void ControlGoToPosClean::drawInInterface() {
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData = {{{}, {}}};
    for (auto &displayAll : displayData) {
        displayColorData.push_back(displayAll);
    }
    for (auto &displayPath : path) {
        displayColorData.emplace_back(displayPath.pos, Qt::red);
    }
    rtt::ai::interface::Drawer::setGoToPosLuThPoints(robotID, displayColorData);
}

/// draw a cross in the interface
void ControlGoToPosClean::drawCross(Vector2 &pos, QColor color) {
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
void ControlGoToPosClean::drawPoint(Vector2 &pos, QColor color) {
    displayData.emplace_back(pos, color);
}

/// add a child to the current path
void ControlGoToPosClean::PathPoint::addChild(std::shared_ptr<PathPoint> &newChild) {
    children.push_back(newChild);
}
/// add multiple children to the current path
void ControlGoToPosClean::PathPoint::addChildren(std::vector<std::shared_ptr<PathPoint>> &newChildren) {
    children.insert(children.end(), newChildren.begin(), newChildren.end());
}

/// check if a branch already has the target
bool ControlGoToPosClean::PathPoint::branchHasTarget(const Vector2 &target) {

    for (const auto &child : children) {
        if (child->currentTarget == target) {
            return true;
        }
    }
    return false;
}

/// check if ANY branch already has that target
bool ControlGoToPosClean::PathPoint::anyBranchHasTarget(const Vector2 &target) {
    auto root = backTrack(0.0);
    return root->anyChildHasTarget(target);

}

/// check if ANY child already has that target
bool ControlGoToPosClean::PathPoint::anyChildHasTarget(const Vector2 &target) {
    for (const auto &child : children) {
        if ((child->currentTarget - target).length() < 0.15 || child->anyChildHasTarget(target)) {
            return true;
        }
    }
    return false;
}

/// check if ANY parent already has that target
bool ControlGoToPosClean::PathPoint::anyParentHasTarget(const Vector2 &target) {
    if (parent) {
        if (parent->currentTarget == target || parent->anyParentHasTarget(target)) {
            return true;
        }
    }
    return false;
}

/// check if a collision is occuring
bool ControlGoToPosClean::PathPoint::isCollision(Vector2 target, double distance) {
    return (target - pos).length() < distance;
}

}// control
}// ai
}// rtt