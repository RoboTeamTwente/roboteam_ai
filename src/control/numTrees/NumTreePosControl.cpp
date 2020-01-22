//
// Created by thijs on 25-5-19.
//

#include "control/RobotCommand.h"
#include "world/Robot.h"
#include "world/Ball.h"
#include <queue>
#include <interface/api/Input.h>
#include "world/Field.h"
#include "control/numTrees/NumTreePosControl.h"
#include "control/numTrees/PathPoint.h"
#include "control/numTrees/Collision.h"
#include "control/ControlUtils.h"

namespace rtt {
namespace ai {
namespace control {

NumTreePosControl::NumTreePosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        :BasicPosControl(avoidBall, canMoveOutsideField, canMoveInDefenseArea) { }

/// Clears data and resets variables
void NumTreePosControl::clear() {
    path.clear();
}

/// return the velocity command using two PIDs based on the current position and velocity of the robot compared to the
/// position and velocity of the calculated path
RobotCommand NumTreePosControl::computeCommand(const Vector2 &exactTargetPos) {
    RobotCommand target;
    double goToTimeInFuture = 0.4;
    auto targetPathPoint = static_cast<unsigned long>(goToTimeInFuture/DT);
    if (path.size() < targetPathPoint) {
        target.pos = exactTargetPos;
        target.vel = exactTargetPos - robot->pos;
        target.angle = target.vel.toAngle();
        return target;
    }

    target.pos = robot->pos + (path[targetPathPoint].pos - robot->pos)*1.0;
    target.vel = path[targetPathPoint].vel; // Velocity is not used currently!!
    target.angle = (target.pos - robot->pos).angle();

    interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, {target.pos}, Qt::green, robot->id,
            interface::Drawing::DOTS, 12, 12);
    interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, {target.pos, target.pos + target.vel*0.4}, Qt::red,
            robot->id,
            interface::Drawing::LINES_CONNECTED);
    return target;
}

RobotCommand NumTreePosControl::getRobotCommand(world::World * world, world::Field * field, const RobotPtr &robotPtr,
        const Vector2 &targetPos, const Angle &targetAngle, bool illegalPositions) {
    this->world = world;
    this->field = field;

    bool tempAllow = allowIllegalPositions;
    allowIllegalPositions = illegalPositions;
    RobotCommand robotCommand = NumTreePosControl::getRobotCommand(world, field, robotPtr, targetPos, targetAngle);
    allowIllegalPositions = tempAllow;
    return robotCommand;
}

RobotCommand NumTreePosControl::getRobotCommand(world::World * world, world::Field * field, const RobotPtr &robotPtr,
        const Vector2 &targetPos, bool illegalPositions) {
    this->world = world;
    this->field = field;

    Angle defaultAngle = 0;
    return getRobotCommand(world, field, robotPtr, targetPos, defaultAngle, illegalPositions);
}

/// finds a path using a numeric model
RobotCommand NumTreePosControl::getRobotCommand(world::World * world, world::Field * field, const RobotPtr &robotPtr,
        const Vector2 &targetPos, const Angle &targetAngle) {
        this->world = world;
        this->field = field;

    DT = 0.3/rtt::ai::GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel;
    if (DT > 0.12) DT = 0.12;
    if (DT < 0.06) DT = 0.06;

    // check if the robot exists
    if (! robotPtr) {
        std::cerr << "NO ROBOT IN NUMTREES!!" << std::endl;
        return {};
    }

    // make a copy of the robot
    robot = std::make_shared<world::Robot>(world::Robot(*robotPtr));
    //ros::Time begin = ros::Time::now();

    // Check if the current path is still valid, if not, recalculate
    bool nicePath = true;
    bool recalculate = doRecalculatePath(targetPos);
    if (recalculate) {
        finalTargetPos = targetPos;

        if (Vector2(robot->vel).length() > Constants::MAX_VEL_CMD()) {
            nicePath = false;
            if (InterfaceValues::showDebugNumTreeInfo())
                std::cout << "ROBOT " << robot->id << ": is moving too fast, check world_state?" << std::endl;
        }
        else {
            // Calculate new path
            tracePath();
            if (path.empty()) {
                nicePath = false;
            }
        }
    }
//    ros::Time end = ros::Time::now();
//    if (InterfaceValues::showDebugNumTreeTimeTaken() && InterfaceValues::showFullDebugNumTreeInfo()) {
//        std::cout << "ROBOT " << robot->id << ": GoToPosClean tick took: " <<
//                  (end - begin).toNSec()*0.000001 << " ms" << std::endl;
//    }

    // draw
    std::vector<Vector2> drawpoints = {};
    for (int i = path.size()-1; i >= 0; i--) {
        drawpoints.push_back(path[i].pos);
    }
    interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, triedPaths, Qt::red, robot->id,
            interface::Drawing::DOTS, 3, 3);
    interface::Input::drawData(interface::Visual::PATHFINDING, drawpoints, Qt::green, robot->id,
            interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, {targetPos}, Qt::yellow, robot->id,
            interface::Drawing::CIRCLES, 8, 8, 6);
    interface::Input::drawData(interface::Visual::PATHFINDING, {finalTargetPos}, Qt::darkYellow, robot->id,
            interface::Drawing::CIRCLES, 8, 8, 6);

    // check if we have a nice path.
    if (! nicePath) {
        path.clear();
    }
    RobotCommand command = computeCommand(targetPos);
    return controlWithPID(robotPtr, command);
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

    path.clear();
    triedPaths.clear();

    // create a new path with a root
    std::priority_queue<PathPointer, std::vector<PathPointer>, decltype(compAStar)> pathQueue(compAStar);
    PathPointer root = std::make_shared<PathPoint>();
    root->currentTarget = finalTargetPos;
    root->pos = robot->pos;
    root->vel = robot->vel;
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
            if (InterfaceValues::showDebugNumTreeInfo()) {
                std::cout << "ROBOT " << robot->id << ": Tick took too long!" << std::endl;
            }
            PathPointer bestPath = pathQueue.top();
            pathQueue.pop();
            while (! pathQueue.empty()) {
                PathPointer pathPointer = pathQueue.top();
                if ((finalTargetPos - pathPointer->pos).length2() < (finalTargetPos - bestPath->pos).length2()) {
                    bestPath = pathPointer;
                }
                pathQueue.pop();
            }
            path = backTrackPath(bestPath, root);
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
    if (InterfaceValues::showDebugNumTreeInfo()) {
        std::cout << "ROBOT " << robot->id << ": reached end of while loop, no path found" << std::endl;
    }
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

    // Copy parent
    PathPointer newPoint = std::make_shared<PathPoint>();
    newPoint->parent = oldPoint;
    oldPoint->addChild(newPoint);
    newPoint->t = oldPoint->t + DT;
    newPoint->currentTarget = subTarget;
    newPoint->finalTarget = finalTargetPos;
    newPoint->pos = oldPoint->pos;
    newPoint->vel = oldPoint->vel;
    triedPaths.push_back(newPoint->pos);

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

Collision NumTreePosControl::getCollision(const PathPointer &point, double collisionRadius) {
    // check collisions with robots, the ball, out of field, defense area (and more if needed)
    double futureTime = point->t;

    // Collision with Robots
    auto allRobots = world->getAllRobots();
    for (auto &r : allRobots) {
        r = world->getFutureRobot(r, futureTime);
    }
    auto robotCollision = getRobotCollision(point, allRobots, collisionRadius);
    if (robotCollision.isCollision) return robotCollision;

    // Collision with Ball
    auto ball = world->getFutureBall(futureTime);
    auto ballCollision = getBallCollision(point, ball);
    if (ballCollision.isCollision) return ballCollision;

    // Collision with Edge of Field
    auto fieldCollision = getFieldCollision(point);
    if (fieldCollision.isCollision) return fieldCollision;

    // Collision with Defense Area
    auto defenseAreaCollision = getDefenseAreaCollision(point);
    if (defenseAreaCollision.isCollision) return defenseAreaCollision;

    auto goalCollision = getGoalCollision(point);
    if (goalCollision.isCollision) return goalCollision;

    if (GameStateManager::getCurrentGameState().strategyName == "ball_placement_us_strategy"
    || GameStateManager::getCurrentGameState().strategyName == "ball_placement_them_strategy") {
        auto ballPlacementCollision = getBallPlacementCollision(point);
        if (ballPlacementCollision.isCollision) return ballPlacementCollision;
    }

    return {};
}

Collision NumTreePosControl::getRobotCollision(
        const PathPointer &point, const std::vector<RobotPtr> &robots, double distance) {

    // for all robots check if the distance to collisionPos is smaller than the set distance
    Collision collision = {};
    auto currentRobotCollision = currentCollisionWithRobot.getCollisionRobot();
    auto currentFinalTargetCollision = currentCollisionWithFinalTarget.getCollisionRobot();

    for (auto &r : robots) {
        // cant collide with itself
        if (r->id == this->robot->id && r->team == this->robot->team) continue;
        // don't look at the robot it is colliding with already
        if (r->id == currentRobotCollision->id && r->team == currentRobotCollision->team) continue;
        // don't look at the final target if the robot is colliding
        if (r->id == currentFinalTargetCollision->id && r->team == currentFinalTargetCollision->team) continue;

        if (point->isCollision(r->pos, distance)) {
            collision.setCollisionRobot(r, distance);
            return collision;
        }
    }
    return collision;
}

Collision NumTreePosControl::getBallCollision(const PathPointer &point, const PosController::BallPtr &ball) {
    Collision collision = {};
    if (currentCollisionWithRobot.getCollisionBall()->getVisible()) return collision;
    if (currentCollisionWithFinalTarget.getCollisionBall()->getVisible()) return collision;

    double avoidBallDistance = getAvoidBallDistance();
    if (point->isCollision(ball->getPos(), avoidBallDistance)) {
        collision.setCollisionBall(ball, avoidBallDistance);
        return collision;
    }
    return collision;
}

Collision NumTreePosControl::getFieldCollision(const PathPointer &point) {
    Collision collision = {};
    bool canMoveOutOfField = currentCollisionWithRobot.getCollisionFieldPos() != Vector2() ||
                             currentCollisionWithFinalTarget.getCollisionFieldPos() != Vector2() ||
                             getCanMoveOutOfField(robot->id);

    double margin = canMoveOutOfField ? -0.30 + Constants::ROBOT_RADIUS() : Constants::ROBOT_RADIUS();
    if (! world::field->pointIsInField(point->pos, margin)) {
        collision.setFieldCollision(point->pos, 0.2);
    }
    return collision;
}

Collision NumTreePosControl::getDefenseAreaCollision(const PathPointer &point) {
    Collision collision = {};
    if (currentCollisionWithRobot.getCollisionDefenseAreaPos() != Vector2()) return collision;
    if (currentCollisionWithFinalTarget.getCollisionDefenseAreaPos() != Vector2()) return collision;

    if (! getCanMoveInDefenseArea(robot->id)) {
        auto margin = Constants::ROBOT_RADIUS();
        bool isInOurDefenseArea = field->pointIsInDefenceArea(point->pos, true, margin, false);
        bool isInTheirDefenseArea = field->pointIsInDefenceArea(point->pos, false, margin, false);
        if (isInOurDefenseArea || isInTheirDefenseArea) {
            double defenseAreaX = point->pos.x < 0 ? field->get_field().get(LEFT_PENALTY_LINE).begin.x:
                                  field->get_field().get(RIGHT_PENALTY_LINE).begin.x;
            collision.setDefenseAreaCollision(point->pos, (fabs(defenseAreaX - point->pos.x) + margin)*1.1);
            return collision;
        }
    }
    return collision;
}

Collision NumTreePosControl::getGoalCollision(const NumTreePosControl::PathPointer &point) {
    Collision collision = {};
    if (currentCollisionWithRobot.getCollisionGoalPos() != Vector2()) return collision;
    if (currentCollisionWithFinalTarget.getCollisionGoalPos() != Vector2()) return collision;

    bool collidesWithOurGoal = field->getGoalArea(true, Constants::ROBOT_RADIUS(), true).contains(point->pos);
    bool collidesWithTheirGoal = field->getGoalArea(false, Constants::ROBOT_RADIUS(), true).contains(point->pos);

    if (collidesWithOurGoal || collidesWithTheirGoal) {
        collision.setGoalCollision(point->pos, field->get_field().get(GOAL_WIDTH) / 2 - fabs(point->pos.y) * 1.1);
    }

    return collision;
}

Collision NumTreePosControl::getBallPlacementCollision(const NumTreePosControl::PathPointer &point) {
    Collision collision = {};
    if (currentCollisionWithRobot.getCollisionBallPlacement() != Vector2()) return collision;
    if (currentCollisionWithFinalTarget.getCollisionBallPlacement() != Vector2()) return collision;

    auto ball = world->getBall();

    Vector2 ballPlacementMarker = rtt::ai::GameStateManager::getRefereeDesignatedPosition();

    if (!interface::Output::usesRefereeCommands()) {
        ballPlacementMarker = rtt::ai::interface::Output::getInterfaceMarkerPosition();
        std::cerr << "GETTING BALLPLACEMENT LOCATION FROM INTERFACE" << std::endl;
    };


    double avoidDist = fmin (ballPlacementMarker.dist(ball->getPos()), 2.0);
    auto shortenedDistance = (ballPlacementMarker - ball->getPos()).stretchToLength(avoidDist);

    bool collidesWithBallPlacement = control::ControlUtils::distanceToLineWithEnds(point->pos, Vector2(ball->getPos()),
            ball->getPos() + shortenedDistance) < 0.5;
    Vector2 diff = (shortenedDistance).rotate(M_PI_2);

    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->getPos() + diff.stretchToLength(0.5),
          ball->getPos() + shortenedDistance + diff.stretchToLength(0.5)}, Qt::darkCyan, -1, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->getPos() - diff.stretchToLength(0.5),
          ball->getPos() + shortenedDistance - diff.stretchToLength(0.5)}, Qt::darkCyan, -1, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::BALLPLACEMENT, {ball->getPos(), ball->getPos() + shortenedDistance},
            Qt::darkCyan, -1, interface::Drawing::REAL_LIFE_CIRCLES, 0.5, 0.5);

    if (collidesWithBallPlacement) {
        double newLocation = (fmax(ball->getPos().dist(point->pos), (ball->getPos() + shortenedDistance).dist(point->pos))) * 1.2;
        collision.setBallPlacementCollision(point->pos, newLocation);
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

    Vector2 rightTargetPosition = collisionPoint->pos +
            Vector2(deltaPosition.y, - deltaPosition.x).stretchToLength(- collisionRadius*sqrt(factor)*1.2);

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

void NumTreePosControl::checkInterfacePID() {
    auto newPid = interface::Output::getNumTreePid();
    updatePid(newPid);
}

RobotCommand NumTreePosControl::getRobotCommand(world::World * world, world::Field * field, const RobotPtr &robotPtr, const Vector2 &targetPos) {
    this->world = world;
    this->field = field;
    Angle defaultAngle;
    return NumTreePosControl::getRobotCommand(world, field, robotPtr, targetPos, defaultAngle);
}

/// finds a reason to calculate a new path (possible reasons are: no path calculated yet, final target moved,
/// robot is too far from path or another robot is colliding with current path
bool NumTreePosControl::doRecalculatePath(const Vector2 &targetPos) {
    if (checkChangeInMaxRobotVel()) return true;
    if (checkEmptyPath()) return true;

    double maxDeviation = 0.3;
    if (checkIfTargetMoved(maxDeviation, targetPos)) return true;
    if (checkIfAtEndOfPath(maxDeviation, targetPos)) return true;
    if (checkIfTooFarFromCurrentPath(maxDeviation, targetPos)) return true;

    if (checkCurrentRobotCollision()) return true;

    return checkIfRobotWillCollideFollowingThisPath();
}

bool NumTreePosControl::checkCurrentRobotCollision() {
    Collision previousCollisionWithRobot = currentCollisionWithRobot;
    Collision previousCollisionWithFinalTarget = currentCollisionWithFinalTarget;

    currentCollisionWithRobot = Collision();
    currentCollisionWithFinalTarget = Collision();

    PathPointer realRobotPoint = std::make_shared<PathPoint>();
    realRobotPoint->pos = robot->pos;
    realRobotPoint->vel = robot->vel;
    realRobotPoint->t = 0;
    currentCollisionWithRobot = getCollision(realRobotPoint, DEFAULT_ROBOT_COLLISION_RADIUS);

    PathPointer finalTargetPoint = std::make_shared<PathPoint>();
    finalTargetPoint->pos = finalTargetPos;
    finalTargetPoint->vel = Vector2();
    finalTargetPoint->t = 0;
    currentCollisionWithFinalTarget = getCollision(finalTargetPoint, DEFAULT_ROBOT_COLLISION_RADIUS);

    if (! allowIllegalPositions) {
        if (currentCollisionWithFinalTarget.getCollisionType() == Collision::DEFENSE_AREA ||
                currentCollisionWithRobot.getCollisionType() == Collision::DEFENSE_AREA) {

            finalTargetPos = ControlUtils::projectPositionToOutsideDefenseArea(
                    finalTargetPos, Constants::ROBOT_RADIUS()*1.1);

            currentlyAvoidingDefenseArea = finalTargetPos == currentlyAvoidingDefenseAreaPosition;
            if (! currentlyAvoidingDefenseArea) {
                currentlyAvoidingDefenseAreaPosition = finalTargetPos;
                currentlyAvoidingDefenseArea = true;

                currentCollisionWithFinalTarget = {};
                currentCollisionWithRobot = {};
                pathHasRobotCollision = false;

                path.clear();
                return true;
            }
        }
        else {
            currentlyAvoidingDefenseArea = false;
            currentlyAvoidingDefenseAreaPosition = Vector2();
        }
    }

    bool collision = currentCollisionWithRobot.isCollision &&
            previousCollisionWithRobot.getCollisionType() != currentCollisionWithRobot.getCollisionType() &&
            currentCollisionWithFinalTarget.isCollision &&
            previousCollisionWithFinalTarget.getCollisionType() != currentCollisionWithFinalTarget.getCollisionType() &&
            ! pathHasRobotCollision;

    if (collision) {
        if (InterfaceValues::showDebugNumTreeInfo()) {
            std::string s = currentCollisionWithRobot.collisionTypeToString();
            std::cout << "ROBOT " << robot->id << ": is too close to " << s <<
                      "-> trying to make a path anyways <-" << std::endl;
        }
        path.clear();
        pathHasRobotCollision = false;
        return true;
    }

    pathHasRobotCollision = false;
    return false;
}

bool NumTreePosControl::checkEmptyPath() {
    if (path.empty()) {
        if (InterfaceValues::showFullDebugNumTreeInfo()) {
            std::cout << "ROBOT " << robot->id << ": no path, recalculating" << std::endl;
        }
        return true;
    }
    return false;
}

bool NumTreePosControl::checkIfTargetMoved(double maxTargetDeviation, const Vector2 &targetPos) {
    // if the target moved too much
    if ((finalTargetPos - targetPos).length2() > maxTargetDeviation*maxTargetDeviation) {
        if (InterfaceValues::showFullDebugNumTreeInfo()) {
            std::cout << "ROBOT " << robot->id << ": target moved too much, recalculating" << std::endl;
        }
        return true;
    }
    return false;
}

bool NumTreePosControl::checkIfAtEndOfPath(double maxTargetDeviation, const Vector2 &targetPos) {
    // if the end of the path is met
    if (path.size() < static_cast<unsigned int>(1.01 + 1.0/DT)) {
        if ((path[path.size() - 1].pos - targetPos).length() > maxTargetDeviation) {
            if (InterfaceValues::showFullDebugNumTreeInfo()) {
                std::cout << "ROBOT " << robot->id << ": reached end of path segment, recalculating" << std::endl;
            }
            return true;
        }
    }
    return false;
}

bool NumTreePosControl::checkIfTooFarFromCurrentPath(double maxTargetDeviation, const Vector2 &vector2) {
    // check if the robot is too far from its current path
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
        if (InterfaceValues::showDebugNumTreeInfo()) {
            std::cout << "ROBOT " << robot->id << ": is too far from current path, recalculating" << std::endl;
        }
        return true;
    }
    for (int i = 0; i < static_cast<int>(currentIndex); i ++) {
        auto p = path[i];
        triedPaths.push_back(p.pos);
    }
    path.erase(path.begin(), path.begin() + currentIndex);
    return false;
}

bool NumTreePosControl::checkIfRobotWillCollideFollowingThisPath() {
    // check if there is a collision for any of the upcoming points in the path
    for (auto pathPoint : path) {
        if (getCollision(std::make_shared<PathPoint>(pathPoint), 0.8*DEFAULT_ROBOT_COLLISION_RADIUS).isCollision) {
            if (InterfaceValues::showDebugNumTreeInfo()) {
                std::cout << "ROBOT " << robot->id << ": another robot will collide with ours when "
                                                      "following this path, recalculating" << std::endl;
            }
            return true;
        }
    }
    return false;
}
bool NumTreePosControl::checkChangeInMaxRobotVel() {
    double newMaxRobotVel = rtt::ai::GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel;
    if (newMaxRobotVel != currentMaxRobotVel) {
        currentMaxRobotVel = newMaxRobotVel;
        return true;
    }
    return false;
}

const Collision &NumTreePosControl::getCurrentCollisionWithRobot() const {
    return currentCollisionWithRobot;
}

const Collision &NumTreePosControl::getCurrentCollisionWithFinalTarget() const {
    return currentCollisionWithFinalTarget;
}


} // control
} // ai
} // rtt