//
// Created by robzelluf on 5/27/19.
//

#include "MidFieldCoach.h"

namespace rtt {
namespace ai {
namespace coach {

MidFieldCoach g_midFieldCoach;

//TODO: Remove midfielders that are not midfielders anymore

void MidFieldCoach::addMidFielder(RobotPtr &thisRobot) {
    Vector2 target = thisRobot->pos;
    currentMidfielders.push_back(thisRobot);
    targetPositions[thisRobot->id] = target;
}

MidFieldCoach::Target MidFieldCoach::getTargetPosition(MidFieldCoach::RobotPtr &thisRobot) {
    // Check if already having an opponent as target
    if(targetRobotsToHarass.find(thisRobot->id) != targetRobotsToHarass.end()) {
        RobotPtr opponent = targetRobotsToHarass[thisRobot->id];
        if(validOpponent(opponent)) {
            return harassRobot(thisRobot, opponent, HARASS_DEFENSIVE);
        } else {
            targetRobotsToHarass.erase(thisRobot->id);
        }
    }

    // If no opponent, or opponent not valid anymore, pick new target
    RobotPtr opponent = findRobotToHarass(thisRobot);

    // If there is a opponent to harass
    if(opponent) {
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
                return standFree(thisRobot);
            }
        }
    } else {
        return standFree(thisRobot);
    }

}

bool MidFieldCoach::validOpponent(const RobotPtr& opponent) {
    if (abs(opponent->pos.x) > DISTANCE_FROM_MIDDLE_LINE) {
        return false;
    }

    return world::field->pointIsInField(opponent->pos);

}

void MidFieldCoach::removeMidFielder(MidFieldCoach::RobotPtr &thisRobot) {
    currentMidfielders.erase(std::remove(currentMidfielders.begin(), currentMidfielders.end(), thisRobot), currentMidfielders.end());

    if(targetPositions.find(thisRobot->id) != targetPositions.end()) {
        targetPositions.erase(thisRobot->id);
    }

    if(targetRobotsToHarass.find(thisRobot->id) != targetRobotsToHarass.end()) {
        targetRobotsToHarass.erase(thisRobot->id);
    }
}

MidFieldCoach::Target MidFieldCoach::blockPass(const RobotPtr &thisRobot, const RobotPtr &opponent, const BallPtr &ball) const {
    Target target;
    Vector2 projectionPoint = thisRobot->pos.project(ball->pos, opponent->pos);
    if (control::ControlUtils::isPointProjectedOnLineSegment(projectionPoint, opponent->pos, ball->pos)) {
        target.targetPosition = projectionPoint;
    } else {
        target.targetPosition = ball->pos;
    }
    return target;
}

MidFieldCoach::Target MidFieldCoach::harassRobot(const RobotPtr &thisRobot, const RobotPtr &opponent, HarassType harassType) const {
    Target target;
    target.targetRobot = opponent->id;
    if (opponent->vel.length() < MIN_OPPONENT_VELOCITY) {
        if(harassType == HARASS_DEFENSIVE) {
            target.targetPosition = opponent->pos - Vector2{DEFAULT_HARASS_DISTANCE, 0};
        } else {
            target.targetPosition = opponent->pos + Vector2{DEFAULT_HARASS_DISTANCE, 0};
        }
    } else {
        Vector2 futureOpponentPos = opponent->pos + opponent->vel * HARASSER_SECONDS_AHEAD;
        Vector2 projectionPoint = thisRobot->pos.project(opponent->pos, futureOpponentPos);

        // Check if the projection is actually between the opponent and it's future position
        // If not, move to the future position
        if (control::ControlUtils::isPointProjectedOnLineSegment(projectionPoint, opponent->pos,
                                                                 futureOpponentPos)) {
            if ((projectionPoint - thisRobot->pos).length() < STAND_STILL_DISTANCE) {
                target.targetPosition = thisRobot->pos;
            } else {
                target.targetPosition = projectionPoint;
            }
        } else {
            target.targetPosition = futureOpponentPos;
        }
    }
    return target;
}

MidFieldCoach::RobotPtr MidFieldCoach::findRobotToHarass(const RobotPtr& thisRobot) {
    RobotPtr closestRobot = nullptr;
    auto shortestDistance = DBL_MAX;

    // Loop over all opponents to find a opponent to harass
    for(const auto &opponent : world::world->getThem()) {
        // If opponent is not valid, ignore it
        if (!validOpponent(opponent)) continue;

        bool alreadyBeingHarassed =false;
        for(const auto &currentOpponent : targetRobotsToHarass) {
            if (opponent->id == currentOpponent.second->id) {
                alreadyBeingHarassed = true;
                break;
            }
        }

        if(!alreadyBeingHarassed) {
            double distance = (opponent->pos - thisRobot->pos).length();
            if (distance < shortestDistance) {
                closestRobot = opponent;
                shortestDistance = distance;
            }
        }
    }

    return closestRobot;
}

MidFieldCoach::Target MidFieldCoach::standFree(const RobotPtr &thisRobot) {
    Target target;
    target.targetRobot = -1;
    target.targetPosition = thisRobot->pos;
    return target;
}

MidFieldCoach::HarassType MidFieldCoach::getHarassType(const RobotPtr& thisRobot, const RobotPtr& opponent) {
    auto ball = world::world->getBall();
    BallPossession ballPossession;
    auto possession = ballPossession.getPossession();

    // Check if the opponent is left of us (is being offensive)
    if (opponent->pos.x < thisRobot->pos.x) {

        // Check if ball is on our left
        if (ball->pos.x < thisRobot->pos.x) {
            if(possession == BallPossession::OURBALL) {
                return STAND_FREE;
            } else {
                return HARASS_DEFENSIVE;
            }
            // Else, the ball is on our right
        } else {
            if(ballPossession.getPossession() == BallPossession::OURBALL) {
                return HARASS_OFFENSIVE;
            } else {
                return BLOCK_PASS;
            }
        }

        // Else, the opponent is on our right (being defensive or about to offend)
    } else {
        if(possession == BallPossession::OURBALL) {
            return HARASS_OFFENSIVE;
        } else {
            return HARASS_DEFENSIVE;
        }
    }
}

MidFieldCoach::Target MidFieldCoach::getBall(RobotPtr &thisRobot, const RobotPtr& opponent) {
    Target target;
    target.targetRobot = opponent->id;
    target.targetPosition = world::world->getBall()->pos;
    return target;
}

} //coach
} //ai
} //rtt