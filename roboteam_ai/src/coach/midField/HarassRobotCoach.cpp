//
// Created by thijs on 12-4-19.
//

#include "HarassRobotCoach.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/world/World.h>
#include "../../world/WorldData.h"
#include <algorithm> //sort

namespace rtt {
namespace ai {
namespace coach {

HarassRobotCoach g_harassRobotCoach;

// update harass-targetPosition based on current position and the position of other robots in the same tactic
// myIndex is used to keep track of the position of this robot in the static array
Vector2 HarassRobotCoach::getHarassPosition(const RobotPtr &thisRobot, int &myIndex) {

    Vector2 currentLocation = thisRobot->pos;
    auto robotWithBall = world::world->whichRobotHasBall();
    if (robotWithBall) bestXPos = robotWithBall->team == world::Robot::them ? - 1.2 : 1.2;
    else bestXPos = 0.0;

    // initialize
    if (myIndex == - 1) {
        return initialize(currentLocation, myIndex);
    }

    // update location
    currentRobotPositions[myIndex] = currentLocation;

    // get one of the robots to get/defend the ball
    auto ball = world::world->getBall();
    if (ball) {
        if (robotWithBall) {

            // they have ball
            if (robotWithBall->team == world::Robot::them) {
                // if this robot is closest to the robot with ball, harass that robot
                int bestIndex = getRobotIndexCloseToEnemyRobot(robotWithBall);
                if (bestIndex == myIndex) {
                    return harassRobot(myIndex, robotWithBall->id, false);
                }

                // else if this robot already has another robot assigned
                if (targetRobotsToHarass[myIndex]) {
                    return harassRobot(myIndex, targetRobotsToHarass[myIndex]->id);
                }

                // else, get a new robot to harass
                std::vector<Robot> theirRobots = world::world->getThem();
                std::sort(theirRobots.begin(), theirRobots.end(),
                        [ball](const Robot &a, const Robot &b) -> bool {
                          // sorts by distance of enemy robot to this robot
                          return (a.pos - ball->pos).length2() < (b.pos - ball->pos).length2();
                        });

                for (auto &robot : theirRobots) {
                    bool robotIsHarassed = false;
                    for (int i = 0; i < targetRobotsToHarass.size(); i++) {
                        if (i == myIndex) continue;

                        auto &harassTarget = targetRobotsToHarass[i];
                        if (harassTarget) {
                            if (robot.id == harassTarget->id) {
                                robotIsHarassed = true;
                                break;
                            }
                        }
                    }
                    if (! robotIsHarassed) return harassRobot(myIndex, robot.id);
                }

                // can't get a robot to harass, remove my targetRobot if it was there
                targetRobotsToHarass[myIndex] = RobotPtr(nullptr);
            }

            // we have ball
            else {
                return standFree(thisRobot, myIndex, robotWithBall);
            }

        }
        // nobody has ball
        else {
            bool canIContest = true; //TODO: check if this robot can contest the ball
            if (canIContest) {
                //contestBall();
            }
            else {
                //doWhatever();
            }
        }

    }

    // if not in the right x-location yet, go there first
    if (abs(currentLocation.x - bestXPos) > 0.5) {
        Vector2 target = {bestXPos, currentLocation.y};
        targetRobotPositions[myIndex] = target;
        return target;
    }

    // if you have nothing else to do, just don't get too close to other harass-robots
    double maxDistance = 0.8;
    for (int i = 0; i < currentRobotPositions.size(); i ++) {
        if (i == myIndex) continue;

        auto &robotPos = currentRobotPositions[i];
        if ((robotPos - currentLocation).length2() < maxDistance*maxDistance) {
            Vector2 target = {currentLocation.x, currentLocation.y +
                    (double) (robotPos.y > currentLocation.y ? maxDistance*- 1.2 : maxDistance*1.2)};
            targetRobotPositions[myIndex] = target;
            return target;
        }
    }

    return currentLocation;
}

int HarassRobotCoach::getRobotIndexCloseToEnemyRobot(const world::World::RobotPtr &enemyRobot) const {
    double closestDistanceSquared = 9e9;
    int bestIndex = - 1;
    for (int i = 0; i < currentRobotPositions.size(); i ++) {
        auto &robotPos = currentRobotPositions[i];
        double lengthSquared = (robotPos - enemyRobot->pos).length2();
        if (lengthSquared < closestDistanceSquared) {
            closestDistanceSquared = lengthSquared;
            bestIndex = i;
        }
    }
    return bestIndex;
}

Vector2 HarassRobotCoach::harassRobot(int myIndex, int id, bool stayInMidField) {
    // (re)set the robot harassed to the robot with that id
    targetRobotsToHarass[myIndex] = RobotPtr(nullptr);
    RobotPtr robotToHarass = world::world->getRobotForId(id, false);
    BallPtr ball = world::world->getBall();
    if (! robotToHarass) return {currentRobotPositions[myIndex]};
    if (! ball) return {robotToHarass->pos.x - 0.3, robotToHarass->pos.y};

    // set target
    Vector2 target;
    if (robotToHarass->getDistanceToBall() >= 0.0) {
        target = ball->pos + (ball->pos - robotToHarass->pos).stretchToLength(0.2);
    }
    else {
        double a = 0.74;
        target = robotToHarass->pos*a + ball->pos*(1-a);
        target.x = target.x > bestXPos + 1.0 ? target.x*a + (bestXPos + 1.0)*(1-a) : target.x;
        target.x = target.x < bestXPos - 1.0 ? target.x*a + (bestXPos - 1.0)*(1-a) : target.x;
    }

    targetRobotPositions[myIndex] = target;
    targetRobotsToHarass[myIndex] = robotToHarass;
    for (int i = 0; i < targetRobotsToHarass.size(); i++) {
        if (i == myIndex) continue;
        auto &targetRobot = targetRobotsToHarass[i];
        if (! targetRobot) continue;

        if (targetRobot->id == robotToHarass->id) {
            targetRobot = RobotPtr(nullptr);
        }
    }
    return target;
}

Vector2 HarassRobotCoach::standFree(const RobotPtr &thisRobot, int myIndex, const RobotPtr &ourRobotWithBall) {

    Vector2 currentLocation = thisRobot->pos;
    Vector2 passLine = ourRobotWithBall->pos - currentLocation;
    Angle passAngle = passLine.toAngle();
    Angle smallestAngle = Angle(M_PI);
    auto enemyRobots = world::world->getThem();
    for (auto &robot : enemyRobots) {
        Vector2 blockLine = ourRobotWithBall->pos - robot.pos;
        Angle blockAngle = blockLine.toAngle();
        Angle deltaAngle = blockAngle - passAngle;
        if (abs(deltaAngle.getAngle()) < abs(smallestAngle.getAngle())) {
            auto distToLine = control::ControlUtils::distanceToLine(robot.pos, ourRobotWithBall->pos, currentLocation);
            auto distToLineWithEnds = control::ControlUtils::distanceToLineWithEnds(robot.pos, ourRobotWithBall->pos, currentLocation);
            if (distToLine == distToLineWithEnds) {
                smallestAngle = deltaAngle;
            }
        }
    }
    Vector2 target;
    Vector2 closeRobotPos = Vector2(42.0, 42.0);
    for (auto &otherRobot : world::world->getAllRobots()) {
        if (otherRobot.id == thisRobot->id && otherRobot.team == thisRobot->team) continue;

        if ((otherRobot.pos - currentLocation).length2() < 0.5) {
            closeRobotPos = otherRobot.pos;
        }
    }
    if (closeRobotPos != Vector2(42.0, 42.0)) {
        target = currentLocation + (currentLocation-closeRobotPos).normalize();
    }
    else if (abs(smallestAngle) > 0.25) {
        target = currentLocation;
    }
    else if (smallestAngle.getAngle() > 0.0) {
        target = currentLocation + passLine.rotate(M_PI_2).normalize();
    }
    else {
        target = currentLocation - passLine.rotate(M_PI_2).normalize();
    }

    target.x = target.x < bestXPos ? bestXPos : target.x;
    targetRobotPositions[myIndex] = target;
    targetRobotsToHarass[myIndex] = RobotPtr(nullptr);
    return target;
}

Vector2 HarassRobotCoach::initialize(const Vector2 &currentLocation, int &myIndex) {
    Vector2 target = {bestXPos, currentLocation.y};
    currentRobotPositions.push_back(currentLocation);
    targetRobotPositions.push_back(target);
    targetRobotsToHarass.push_back(RobotPtr(nullptr));
    myIndex = currentRobotPositions.size() - 1; // set my index (side-effect, but works well tm)
    return target;
}

Angle HarassRobotCoach::getHarassAngle(const HarassRobotCoach::RobotPtr &thisRobot, int &myIndex) {
    BallPtr ball = world::world->getBall();

    // if there is a robot to harass
    if (targetRobotsToHarass[myIndex]) {
        auto robotToHarass = targetRobotsToHarass[myIndex];
        if (robotToHarass->hasBall()) {
            return (ball->pos - thisRobot->pos).toAngle();
        }
        else {
            return (robotToHarass->pos - thisRobot->pos).toAngle();
        }
    }

    // else, aim towards the ball
    return (ball->pos - thisRobot->pos).toAngle();
}

} //coach
} //ai
} //rtt
