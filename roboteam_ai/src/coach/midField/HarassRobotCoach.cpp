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
HarassRobotCoach::HarassTarget HarassRobotCoach::getHarassPosition(const RobotPtr &thisRobot) {
    HarassTarget harassTarget;

    //Check if robot already has a robot to harass
    if(targetRobotsToHarass.find(thisRobot->id) != targetRobotsToHarass.end()) {
        RobotPtr opponent = targetRobotsToHarass[thisRobot->id];
        // Check if the opponent is still valid
        if (validOpponent(opponent)) {
            harassTarget.harassRobot = opponent->id;
            harassTarget.harassPosition = harassRobot(thisRobot, targetRobotsToHarass[thisRobot->id]->id);
            return harassTarget;
        }
    }

    // If the above does not return, find a new target to go to




    ball = world::world->getBall();
    auto field = world::field->get_field();
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

    // find something to do (harass robot, block ball line, etc..)
    // if ball is on our side, annoy opponents that want to get to our side
    // if we have the ball, stand free to receive it

    // if ball is on their side, annoy robots that want to go to their side

    // if ball on their side
    if (ball->pos.x > 0) {
        return findRobotToHarass(thisRobot, true);

    // else, ball is on our side
    } else {
        if (ballPossession == analysis::WE_HAVE_BALL) {
            // stand in midfield will just stand free to receive a pass
            HarassTarget harassTarget;
            harassTarget.harassRobot = -1;
            harassTarget.harassPosition = standInMidField(thisRobot);
            return harassTarget;
            //position
        } else {
            return findRobotToHarass(thisRobot, false);
        }
    }
}

int HarassRobotCoach::getRobotIdCloseToEnemyRobot(const world::World::RobotPtr &enemyRobot) const {
    double closestDistanceSquared = 9e9;
    int bestId = - 1;
    for (auto &ourRobot : currentMidfielders) {
        double lengthSquared = (ourRobot->pos - enemyRobot->pos).length2();
        if (lengthSquared < closestDistanceSquared) {
            closestDistanceSquared = lengthSquared;
            bestId = ourRobot->id;
        }
    }
    return bestId;
}

Vector2 HarassRobotCoach::harassRobot(const RobotPtr &thisRobot, int opponentId) {
    Vector2 target;
    RobotPtr robotToHarass = world::world->getRobotForId(opponentId, false);

    if (world::world->theirRobotHasBall(opponentId)) {
        target = ball->pos;
    } else {
        // If ball is on our side, harass




        if (robotToHarass->vel.length() > MINIMUM_HARASS_VELOCITY) {
            Vector2 harassingDistance = {robotToHarass->vel.length()*HARASSER_SECONDS_AHEAD, 0};
            target = robotToHarass->pos + harassingDistance.rotate(robotToHarass->vel.toAngle());
        }
        else {
            target = robotToHarass->pos;
            target.x -= DEFAULT_HARASSING_DISTANCE;
        }

        Vector2 projection = thisRobot->pos.project(robotToHarass->pos, target);
        if ((projection - thisRobot->pos).length() < 0.05) {
            target = robotToHarass->pos;
        }
    }

    return target;
}

HarassRobotCoach::HarassTarget HarassRobotCoach::initialize(RobotPtr &thisRobot) {
    Vector2 target = {bestXPos, thisRobot->pos.y};
    currentMidfielders.push_back(thisRobot);
    targetPositions[thisRobot->id] = target;

    HarassTarget harassTarget;
    harassTarget.harassPosition = target;
    harassTarget.harassRobot = -1;

    return harassTarget;
}

Angle HarassRobotCoach::getHarassAngle(const RobotPtr &thisRobot) {
    ball = world::world->getBall();

    for (const auto& opponent : targetRobotsToHarass) {
        // if there is a robot to harass
        if (thisRobot->id == opponent.first) {
            auto robotToHarass = targetRobotsToHarass[thisRobot->id];
            if (robotToHarass->hasBall()) {
                return (ball->pos - thisRobot->pos).toAngle();
            } else {
                return (robotToHarass->pos - thisRobot->pos).toAngle();
            }
        }
    }

    // else, aim towards the ball
    return (ball->pos - thisRobot->pos).toAngle();
}

HarassRobotCoach::HarassTarget HarassRobotCoach::findRobotToHarass(const RobotPtr &thisRobot, bool goAfterBall) {
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
            int bestId = getRobotIdCloseToEnemyRobot(closestRobotToBall);
            if (bestId == thisRobot->id) {
                if (!robotAlreadyBeingHarassed(thisRobot->id, closestRobotToBall->id)) {
                    targetRobotsToHarass[thisRobot->id] = closestRobotToBall;
                    harassTarget.harassRobot = closestRobotToBall->id;
                    harassTarget.harassPosition = harassRobot(thisRobot, closestRobotToBall->id);
                    return harassTarget;
                }
            }
        }
    }

    // else, harass the opponent closest to the harasser
    if(closestRobotToHarasser) {
        if (! robotAlreadyBeingHarassed(thisRobot->id, closestRobotToHarasser->id)) {
            targetRobotsToHarass[thisRobot->id] = closestRobotToHarasser;
            harassTarget.harassRobot = closestRobotToHarasser->id;
            harassTarget.harassPosition = harassRobot(thisRobot, closestRobotToHarasser->id);
            return harassTarget;
        }
    }

    harassTarget.harassRobot = -1;
    harassTarget.harassPosition = standInMidField(thisRobot);
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

Vector2 HarassRobotCoach::standInMidField(const RobotPtr &thisRobot) {
    Vector2 currentLocation = thisRobot->pos;

    // if not in the right x-location yet, go there first
    if (abs(currentLocation.x - bestXPos) > 0.5) {
        Vector2 target = {bestXPos, currentLocation.y};
        targetPositions[thisRobot->id] = target;
        return target;
    }

    // find the best position to receive a pass
    Vector2 bestReceiveLocation = getBestReceiveLocation(thisRobot);
    targetPositions[thisRobot->id] = bestReceiveLocation;

    // make sure you are not too close to the other midfielders and the side
    return keepDistanceBetweenHarassers(thisRobot);
}

Vector2 HarassRobotCoach::keepDistanceBetweenHarassers(const RobotPtr &thisRobot) {
    for (const auto& ourRobot : currentMidfielders) {
        if (ourRobot->id == thisRobot->id) continue;

        if ((ourRobot->pos - thisRobot->pos).length() < MIN_DISTANCE_BETWEEN_MIDFIELDERS) {
            Vector2 target = {thisRobot->pos.x, thisRobot->pos.y +
                                                 (double) (ourRobot->pos.y > thisRobot->pos.y ?
                                                           MIN_DISTANCE_BETWEEN_MIDFIELDERS * -1.2 :
                                                           MIN_DISTANCE_BETWEEN_MIDFIELDERS * 1.2)};

            targetPositions[thisRobot->id] = target;

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

    return thisRobot->pos;
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

bool HarassRobotCoach::robotAlreadyBeingHarassed(int myId, int opponentID) {
    for (const auto& opponent : targetRobotsToHarass) {
        if (opponent.second) {
            if (opponent.second->id == opponentID && myId != opponent.first) {
                return true;
            }
        }
    }
    return false;
}

bool HarassRobotCoach::validOpponent(RobotPtr opponent) {
    return abs(opponent->pos.x) < DISTANCE_FROM_MIDDLE_LINE;
}

} //coach
} //ai
} //rtt
