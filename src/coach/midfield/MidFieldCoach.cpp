//
// Created by robzelluf on 5/27/19.
//

#include "coach/midfield/MidFieldCoach.h"
#include <world/WorldData.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include "world/Ball.h"
#include "world/Robot.h"

namespace rtt::ai::coach {

MidFieldCoach g_midFieldCoach;

// TODO: Remove midfielders that are not midfielders anymore

void MidFieldCoach::addMidFielder(RobotPtr &thisRobot) {
    Vector2 target = thisRobot->pos;
    currentMidfielders.push_back(thisRobot);
    targetPositions[thisRobot->id] = target;
}

MidFieldCoach::Target MidFieldCoach::getTargetPosition(const Field &field, MidFieldCoach::RobotPtr &thisRobot) {
    // Check if already having an opponent as target
    if (targetRobotsToHarass.find(thisRobot->id) != targetRobotsToHarass.end()) {
        RobotPtr opponent = targetRobotsToHarass[thisRobot->id];
        if (validOpponent(field, opponent)) {
            return harassRobot(thisRobot, opponent, HARASS_DEFENSIVE);
        } else {
            targetRobotsToHarass.erase(thisRobot->id);
        }
    }

    // If no opponent, or opponent not valid anymore, pick new target
    RobotPtr opponent = findRobotToHarass(field, thisRobot);

    // If there is a opponent to harass
    if (opponent) {
        HarassType harassType = getHarassType(thisRobot, opponent);

        switch (harassType) {
            case HARASS_OFFENSIVE: {
                return harassRobot(thisRobot, opponent, HARASS_OFFENSIVE);
            }
            case HARASS_DEFENSIVE: {
                return harassRobot(thisRobot, opponent, HARASS_DEFENSIVE);
            }
            case BLOCK_PASS: {
                return blockPass(thisRobot, opponent, world::world->getBall());
            }
            case BALL: {
                return getBall(thisRobot, opponent);
            }
            case STAND_FREE: {
                return standFree(field, thisRobot);
            }
        }
    } else {
        return standFree(field, thisRobot);
    }
}

bool MidFieldCoach::validOpponent(const Field &field, const RobotPtr &opponent) {
    if (abs(opponent->pos.x) > DISTANCE_FROM_MIDDLE_LINE) {
        return false;
    }
    return FieldComputations::pointIsInField(field, opponent->pos);
}

void MidFieldCoach::removeMidFielder(MidFieldCoach::RobotPtr &thisRobot) {
    currentMidfielders.erase(std::remove(currentMidfielders.begin(), currentMidfielders.end(), thisRobot), currentMidfielders.end());

    if (targetPositions.find(thisRobot->id) != targetPositions.end()) {
        targetPositions.erase(thisRobot->id);
    }

    if (targetRobotsToHarass.find(thisRobot->id) != targetRobotsToHarass.end()) {
        targetRobotsToHarass.erase(thisRobot->id);
    }
}

MidFieldCoach::Target MidFieldCoach::blockPass(const RobotPtr &thisRobot, const RobotPtr &opponent, const BallPtr &ball) const {
    Target target;
    Vector2 projectionPoint = thisRobot->pos.project(ball->getPos(), opponent->pos);
    if (control::ControlUtils::isPointProjectedOnLineSegment(projectionPoint, opponent->pos, ball->getPos())) {
        target.targetPosition = projectionPoint;
    } else {
        target.targetPosition = ball->getPos();
    }
    return target;
}

MidFieldCoach::Target MidFieldCoach::harassRobot(const RobotPtr &thisRobot, const RobotPtr &opponent, HarassType harassType) const {
    Target target;
    target.targetRobot = opponent->id;
    if (opponent->vel.length() < MIN_OPPONENT_VELOCITY) {
        target = harassSlowRobot(opponent, harassType, target);
    } else {
        target = harassFastRobot(thisRobot, opponent, target);
    }
    return target;
}

MidFieldCoach::Target &MidFieldCoach::harassFastRobot(const MidFieldCoach::RobotPtr &thisRobot, const MidFieldCoach::RobotPtr &opponent, MidFieldCoach::Target &target) const {
    Vector2 futureOpponentPos = opponent->pos + opponent->vel * HARASSER_SECONDS_AHEAD;
    Vector2 projectionPoint = thisRobot->pos.project(opponent->pos, futureOpponentPos);

    // Check if the projection is actually between the opponent and it's future position
    // If not, move to the future position
    if (control::ControlUtils::isPointProjectedOnLineSegment(projectionPoint, opponent->pos, futureOpponentPos)) {
        if ((projectionPoint - thisRobot->pos).length() < STAND_STILL_DISTANCE) {
            target.targetPosition = thisRobot->pos;
        } else {
            target.targetPosition = projectionPoint;
        }
    } else {
        target.targetPosition = futureOpponentPos;
    }
    return target;
}

MidFieldCoach::Target &MidFieldCoach::harassSlowRobot(const MidFieldCoach::RobotPtr &opponent, const MidFieldCoach::HarassType &harassType, MidFieldCoach::Target &target) const {
    if (harassType == HARASS_DEFENSIVE) {
        target.targetPosition = opponent->pos - Vector2{DEFAULT_HARASS_DISTANCE, 0};
    } else {
        target.targetPosition = opponent->pos + Vector2{DEFAULT_HARASS_DISTANCE, 0};
    }
    return target;
}

MidFieldCoach::RobotPtr MidFieldCoach::findRobotToHarass(const Field &field, const RobotPtr &thisRobot) {
    RobotPtr closestRobot = nullptr;
    auto shortestDistance = 9e9;

    // Loop over all opponents to find a opponent to harass
    for (const auto &opponent : world::world->getThem()) {
        // If opponent is not valid, ignore it
        if (!validOpponent(field, opponent)) continue;

        bool alreadyBeingHarassed = isRobotAlreadyBeingHarassed(opponent);

        if (!alreadyBeingHarassed) {
            double distance = (opponent->pos - thisRobot->pos).length();
            if (distance < shortestDistance) {
                closestRobot = opponent;
                shortestDistance = distance;
            }
        }
    }

    return closestRobot;
}

bool MidFieldCoach::isRobotAlreadyBeingHarassed(const world::World::RobotPtr &opponent) const {
    bool alreadyBeingHarassed = false;
    for (const auto &currentOpponent : targetRobotsToHarass) {
        if (opponent->id == currentOpponent.second->id) {
            alreadyBeingHarassed = true;
            break;
        }
    }
    return alreadyBeingHarassed;
}

MidFieldCoach::Target MidFieldCoach::standFree(const Field &field, const RobotPtr &thisRobot) {
    Target target;
    target.targetRobot = -1;
    target.targetPosition = calculateNewRobotPosition(field, thisRobot, thisRobot->vel.toAngle());
    return target;
}

MidFieldCoach::HarassType MidFieldCoach::getHarassType(const RobotPtr &thisRobot, const RobotPtr &opponent) {
    auto ball = world::world->getBall();
    BallPossession ballPossession;
    auto possession = ballPossession.getPossession();

    // Check if the opponent is left of us (is being offensive)
    if (opponent->pos.x < thisRobot->pos.x) {
        return getHarassTypeIfOpponentIsOnTheLeft(thisRobot, ball, ballPossession, possession);

        // Else, the opponent is on our right (being defensive or about to offend)
    } else {
        if (possession == BallPossession::OURBALL) {
            return HARASS_OFFENSIVE;
        } else {
            return HARASS_DEFENSIVE;
        }
    }
}

MidFieldCoach::HarassType MidFieldCoach::getHarassTypeIfOpponentIsOnTheLeft(const MidFieldCoach::RobotPtr &thisRobot, const world::World::BallPtr &ball,
                                                                            BallPossession &ballPossession, const BallPossession::Possession &possession) const {
    if (ball->getPos().x < thisRobot->pos.x) {
        if (possession == BallPossession::OURBALL) {
            return STAND_FREE;
        } else {
            return HARASS_DEFENSIVE;
        }
        // Else, the ball is on our right
    } else {
        if (ballPossession.getPossession() == BallPossession::OURBALL) {
            return HARASS_OFFENSIVE;
        } else {
            return BLOCK_PASS;
        }
    }
}

MidFieldCoach::Target MidFieldCoach::getBall(RobotPtr &thisRobot, const RobotPtr &opponent) {
    Target target;
    target.targetRobot = opponent->id;
    target.targetPosition = world::world->getBall()->getPos();
    return target;
}

double MidFieldCoach::calculateStandingFreeScore(const Field &field, const Vector2 &position, const RobotPtr &thisRobot) {
    auto world = world_new::World::instance()->getWorld().value();

    double passLineScore = CoachHeuristics::calculatePassLineScore(position, world);
    double distanceToUsScore = CoachHeuristics::calculateDistanceToClosestTeamMateScore(position, thisRobot->id);
    double distanceToBallScore = CoachHeuristics::calculatePassDistanceToBallScore(field, position, world);

    return passLineScore + distanceToUsScore + distanceToBallScore;
}

Vector2 MidFieldCoach::calculateNewRobotPosition(const Field &field, const RobotPtr &thisRobot, Angle targetAngle) {
    Vector2 bestPosition = targetPositions[thisRobot->id];
    double highestScore = calculateStandingFreeScore(field, bestPosition, thisRobot);

    Angle goldenAngle = 0.01;
    tick++;
    Angle thetaPlus = tick * tick * goldenAngle;
    Angle thetaMinus = -1 * tick * tick * goldenAngle;
    std::vector<Vector2> positions = {bestPosition + thetaPlus.toVector2(1.0 * GRID_SIZE),  bestPosition + thetaPlus.toVector2(2.0 * GRID_SIZE),
                                      bestPosition + thetaPlus.toVector2(4.0 * GRID_SIZE),  bestPosition + thetaMinus.toVector2(1.0 * GRID_SIZE),
                                      bestPosition + thetaMinus.toVector2(2.0 * GRID_SIZE), bestPosition + thetaMinus.toVector2(4.0 * GRID_SIZE)};

    for (const auto &position : positions) {
        if (!FieldComputations::pointIsInField(field, position, 0.20) || abs(position.x) > DISTANCE_FROM_MIDDLE_LINE) continue;
        double score = calculateStandingFreeScore(field, position, thisRobot);
        if (score > highestScore) {
            highestScore = score;
            bestPosition = position;
        }
    }

    targetPositions[thisRobot->id] = bestPosition;
    return bestPosition;
}

}  // namespace rtt::ai::coach