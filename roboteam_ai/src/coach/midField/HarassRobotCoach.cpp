//
// Created by thijs on 12-4-19.
//

#include "HarassRobotCoach.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/world/World.h>
#include "../../world/WorldData.h"
#include <algorithm> //sort
#include <roboteam_ai/src/analysis/AnalysisReport.h>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>

namespace rtt {
namespace ai {
namespace coach {

HarassRobotCoach g_harassRobotCoach;

// update harass-targetPosition based on current position and the position of other robots in the same tactic
// myIndex is used to keep track of the position of this robot in the static array
HarassRobotCoach::HarassTarget HarassRobotCoach::getHarassPosition(const RobotPtr &thisRobot, int &myIndex) {
    ball = world::world->getBall();
    auto field = world::field->get_field();
    Vector2 currentLocation = thisRobot->pos;
    auto report = analysis::GameAnalyzer::getInstance().getMostRecentReport();
    auto ballPossession = report->ballPossession;
    // find the best x position to stand at based on whether we have the ball or they
    if (ballPossession == analysis::WE_HAVE_BALL) {
        bestXPos = 1.2;
    } else if (ballPossession == analysis::THEY_HAVE_BALL) {
        bestXPos = std::min(0.67 * field.field_length, 0.67 * (ball->pos.x + field.field_width));
    } else {
        bestXPos = 0.0;
    }

    // initialize
    if (myIndex == - 1) {
        return initialize(currentLocation, myIndex);
    }

    // update location
    currentRobotPositions[myIndex] = currentLocation;

    // find something to do (harass robot, block ball line, etc..)
    // if ball is on our side, annoy opponents that want to get to our side
    // if we have the ball, stand free to receive it

    // if ball is on their side, annoy robots that want to go to their side

    // if ball on their side
    if (ball->pos.x > 0) {
        return findRobotToHarass(thisRobot, myIndex, true);

    // else, ball is on our side
    } else {
        if (ballPossession == analysis::WE_HAVE_BALL) {
            // stand in midfield will just stand free to receive a pass
            HarassTarget harassTarget;
            harassTarget.harassRobot = -1;
            harassTarget.harassPosition = standInMidField(thisRobot, myIndex);
            return harassTarget;
            //position
        } else {
            return findRobotToHarass(thisRobot, myIndex, false);
        }
    }
}

int HarassRobotCoach::getRobotIndexCloseToEnemyRobot(const world::World::RobotPtr &enemyRobot) const {
    double closestDistanceSquared = 9e9;
    int bestIndex = - 1;
    for (unsigned int i = 0; i < currentRobotPositions.size(); i ++) {
        auto &robotPos = currentRobotPositions[i];
        double lengthSquared = (robotPos - enemyRobot->pos).length2();
        if (lengthSquared < closestDistanceSquared) {
            closestDistanceSquared = lengthSquared;
            bestIndex = i;
        }
    }
    return bestIndex;
}

Vector2 HarassRobotCoach::harassRobot(int myIndex, int id) {
    Vector2 target;
    RobotPtr robotToHarass = world::world->getRobotForId(id, false);

    if (world::world->theirRobotHasBall(id)) {
        target = ball->pos;
    } else {

        if (robotToHarass->vel.length() > MINIMUM_HARASS_VELOCITY) {
            Vector2 harassingDistance = {robotToHarass->vel.length()*HARASSER_SECONDS_AHEAD, 0};
            target = robotToHarass->pos + harassingDistance.rotate(robotToHarass->vel.toAngle());
        }
        else {
            target = robotToHarass->pos;
            target.x -= DEFAULT_HARASSING_DISTANCE;
        }

        Vector2 projection = currentRobotPositions[myIndex].project(robotToHarass->pos, target);
        if ((projection - currentRobotPositions[myIndex]).length() < 0.05) {
            target = robotToHarass->pos;
        }
    }

    return target;
}

HarassRobotCoach::HarassTarget HarassRobotCoach::initialize(const Vector2 &currentLocation, int &myIndex) {
    Vector2 target = {bestXPos, currentLocation.y};
    currentRobotPositions.push_back(currentLocation);
    targetRobotPositions.push_back(target);
    targetRobotsToHarass.push_back(RobotPtr(nullptr));
    myIndex = currentRobotPositions.size() - 1; // set my index (side-effect, but works well tm)

    HarassTarget harassTarget;
    harassTarget.harassPosition = target;
    harassTarget.harassRobot = -1;

    return harassTarget;
}

Angle HarassRobotCoach::getHarassAngle(const HarassRobotCoach::RobotPtr &thisRobot, int &myIndex) {
    ball = world::world->getBall();

    // if there is a robot to harass
    if (myIndex != -1 && targetRobotsToHarass[myIndex]) {
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

HarassRobotCoach::HarassTarget HarassRobotCoach::findRobotToHarass(const RobotPtr &thisRobot, int &myIndex,
        bool goAfterBall) {
    HarassTarget harassTarget;

    // check opponent closest to ball and closest to harasser
    double closestRobotToBallDistance;
    HarassRobotCoach::RobotPtr closestRobotToBall;
    HarassRobotCoach::RobotPtr closestRobotToHarasser;
    setClosestRobots(thisRobot, goAfterBall, closestRobotToBallDistance, closestRobotToBall, closestRobotToHarasser);

    // if robot should go after ball, see if it is close enough and harass it
    if (goAfterBall) {
        // see if opponent is too close to the ball
        if (closestRobotToBallDistance <= TOO_CLOSE_TO_BALL_DISTANCE) {
            // if this robot is closest to the robot with ball, harass that robot
            int bestIndex = getRobotIndexCloseToEnemyRobot(closestRobotToBall);
            if (bestIndex == myIndex) {
                if (!robotAlreadyBeingHarassed(myIndex, closestRobotToBall->id)) {
                    targetRobotsToHarass[myIndex] = closestRobotToBall;
                    harassTarget.harassRobot = closestRobotToBall->id;
                    harassTarget.harassPosition = harassRobot(myIndex, closestRobotToBall->id);
                    return harassTarget;
                }
            }
        }
    }

    // else, harass the opponent closest to the harasser
    if(closestRobotToHarasser) {
        if (! robotAlreadyBeingHarassed(myIndex, closestRobotToHarasser->id)) {
            targetRobotsToHarass[myIndex] = closestRobotToHarasser;
            harassTarget.harassRobot = closestRobotToHarasser->id;
            harassTarget.harassPosition = harassRobot(myIndex, closestRobotToHarasser->id);
            return harassTarget;
        }
    }

    // can't get a robot to harass, remove my targetRobot if it was there
    if (myIndex != -1) {
        targetRobotsToHarass[myIndex] = RobotPtr(nullptr);
    }

    harassTarget.harassRobot = -1;
    harassTarget.harassPosition = standInMidField(thisRobot, myIndex);
    return harassTarget;
}

void HarassRobotCoach::setClosestRobots(const HarassRobotCoach::RobotPtr &thisRobot, bool goAfterBall,
                                        double &closestRobotToBallDistance,
                                        HarassRobotCoach::RobotPtr &closestRobotToBall,
                                        HarassRobotCoach::RobotPtr &closestRobotToHarasser) {

    closestRobotToBallDistance = DBL_MAX;
    auto closestRobotToHarasserDistance = DBL_MAX;
    for (auto robot : world::world->getThem()) {
        // Never harass the keeper
        if (robot->id == GameStateManager::getRefereeData().them.goalie) continue;

        if (abs(robot->pos.x) <= HARASS_THRESHOLD) {
            if (goAfterBall) {
                // check if robot is closest to ball
                double distanceToBall = (ball->pos - robot->pos).length();
                if (distanceToBall < closestRobotToBallDistance) {
                    closestRobotToBallDistance = distanceToBall;
                    closestRobotToBall = robot;
                }
            }

            // check if robot is closest to the harasser
            double distanceToHarasser = (thisRobot->pos - robot->pos).length();
            if (distanceToHarasser < closestRobotToHarasserDistance) {
                closestRobotToHarasserDistance = distanceToHarasser;
                closestRobotToHarasser = robot;
            }
        }
    }
}

Vector2 HarassRobotCoach::standInMidField(const HarassRobotCoach::RobotPtr &thisRobot, int &myIndex) {
    Vector2 currentLocation = thisRobot->pos;

    // if not in the right x-location yet, go there first
    if (abs(currentLocation.x - bestXPos) > 0.5) {
        Vector2 target = {bestXPos, currentLocation.y};
        targetRobotPositions[myIndex] = target;
        return target;
    }

    // find the best position to receive a pass
    Vector2 bestReceiveLocation = getBestReceiveLocation(thisRobot);
    targetRobotPositions[myIndex] = bestReceiveLocation;

    // make sure you are not too close to the other midfielders and the side
    return keepDistanceBetweenHarassers(myIndex, currentLocation);
}

Vector2 HarassRobotCoach::keepDistanceBetweenHarassers(const int &myIndex, Vector2 &currentLocation) {
    for (int i = 0; i < static_cast<int>(currentRobotPositions.size()); i++) {
        if (i == myIndex) continue;

        auto &robotPos = currentRobotPositions[i];
        if ((robotPos - currentLocation).length() < MIN_DISTANCE_BETWEEN_MIDFIELDERS) {
            Vector2 target = {currentLocation.x, currentLocation.y +
                                                 (double) (robotPos.y > currentLocation.y ?
                                                           MIN_DISTANCE_BETWEEN_MIDFIELDERS * -1.2 :
                                                           MIN_DISTANCE_BETWEEN_MIDFIELDERS * 1.2)};

            targetRobotPositions[myIndex] = target;

            if (!world::field->pointIsInField(target, DISTANCE_FROM_SIDES)) {
                auto field = world::field->get_field();
                if (target.y > 0) {
                    target.y = field.field_width / 2 - DISTANCE_FROM_SIDES;
                } else {
                    target.y = -field.field_width / 2 + DISTANCE_FROM_SIDES;
                }
            }

            return target;
        }
    }

    return currentLocation;
}

    Vector2 HarassRobotCoach::getBestReceiveLocation(const HarassRobotCoach::RobotPtr &thisRobot) {
    PassScore passScore;
    Vector2 bestPosition = thisRobot->pos;
    double bestScore = passScore.calculatePassScore(bestPosition);
    double currentX = thisRobot->pos.x;
    double currentY = thisRobot->pos.y;
    for (int yDiff = -GRID_SIZE; yDiff < GRID_SIZE; yDiff++) {
        Vector2 newPos = {currentX, currentY + yDiff * this->GRID_INTERVAL};
        double newScore = passScore.calculatePassScore(newPos);
        if (newScore > bestScore) {
            bestPosition = newPos;
            bestScore = newScore;
        }
    }
    return bestPosition;
}

bool HarassRobotCoach::robotAlreadyBeingHarassed(int myIndex, int opponentID) {
    for (int i = 0; i < targetRobotsToHarass.size(); i++) {
        if (targetRobotsToHarass[i]) {
            if (targetRobotsToHarass[i]->id == opponentID && myIndex != i) {
                return true;
            }
        }
    }
    return false;
}

} //coach
} //ai
} //rtt
