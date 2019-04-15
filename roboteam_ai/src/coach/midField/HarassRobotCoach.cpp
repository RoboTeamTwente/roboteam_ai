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

std::vector<Vector2> HarassRobotCoach::currentRobotPositions = {};
std::vector<Vector2> HarassRobotCoach::targetRobotPositions = {};
std::vector<HarassRobotCoach::RobotPtr> HarassRobotCoach::targetRobotsToHarass = {};

double HarassRobotCoach::bestXPos = 0;

// update harass-targetPosition based on current position and the position of other robots in the same tactic
// myIndex is used to keep track of the position of this robot in the static array
Vector2 HarassRobotCoach::getHarassPosition(const Vector2 &currentLocation, int &myIndex) {

    auto robotWithBall = world::world->whichRobotHasBall();
    if (robotWithBall) bestXPos = world::world->whichRobotHasBall()->team == world::Robot::them ? - 1.2 : 1.2;
    else bestXPos = 0.0;

    // initialize
    if (myIndex == - 1) {
        Vector2 target = {bestXPos, currentLocation.y};
        currentRobotPositions.push_back(currentLocation);
        targetRobotPositions.push_back(target);
        targetRobotsToHarass.push_back(RobotPtr(nullptr));
        myIndex = currentRobotPositions.size() - 1; // set my index (side-effect, but works well tm)
        return target;
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
                double closestDistanceSquared = 9e9;
                int bestIndex = - 1;
                for (int i = 0; i < currentRobotPositions.size(); i ++) {
                    auto &robotPos = currentRobotPositions[i];
                    double lengthSquared = (robotPos - robotWithBall->pos).length2();
                    if (lengthSquared < closestDistanceSquared) {
                        closestDistanceSquared = lengthSquared;
                        bestIndex = i;
                    }
                }
                // if this robot needs to harass the robot with the ball
                if (bestIndex == myIndex) {
                    return harassRobot(myIndex, robotWithBall->id);
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

            }

        }
            // nobody has ball
        else {

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

Vector2 HarassRobotCoach::harassRobot(int myIndex, int id) {
    // (re)set the robot harassed to the robot with that id
    targetRobotsToHarass[myIndex] = RobotPtr(nullptr);
    RobotPtr robotToHarass = world::world->getRobotForId(id, false);
    BallPtr ball = world::world->getBall();
    if (! robotToHarass) return {currentRobotPositions[myIndex]};
    if (! ball) return {robotToHarass->pos.x - 0.3, robotToHarass->pos.y};

    Vector2 target;
    if (robotToHarass->getDistanceToBall() >= 0.0) {
        target = ball->pos + (ball->pos - robotToHarass->pos).stretchToLength(0.2);
    }
    else {
        target = (robotToHarass->pos + ball->pos) * 0.5;
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

} //coach
} //ai
} //rtt
