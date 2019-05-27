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

MidFieldCoach::HarassTarget MidFieldCoach::getTargetPosition(MidFieldCoach::RobotPtr &thisRobot) {
    // Check if already having an opponent as target
    if(targetRobotsToHarass.find(thisRobot->id) != targetRobotsToHarass.end()) {
        RobotPtr opponent = targetRobotsToHarass[thisRobot->id];
        if(validOpponent(opponent)) {
            return harassRobot(thisRobot, opponent);
        } else {
            targetRobotsToHarass.erase(thisRobot->id);
        }
    }

    // If no opponent, or opponent not valid anymore, pick new target
    RobotPtr opponent = findRobotToHarass(thisRobot);

    // If there is a opponent to harass
    if(opponent) {
        return harassRobot(thisRobot, opponent);
    // Else, stand free
    } else {
        HarassTarget harassTarget;
        harassTarget.harassRobot = -1;
        harassTarget.harassPosition = standFree(thisRobot);
        return harassTarget;
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

MidFieldCoach::HarassTarget MidFieldCoach::harassRobot(const RobotPtr& thisRobot, const RobotPtr& opponent) {
    auto ball = world::world->getBall();
    HarassTarget harassTarget;
    harassTarget.harassRobot = -1;
    HarassType harassType = getHarassType(thisRobot, opponent);
    switch (harassType) {
        case HARASS: {
            if(opponent->vel.length() < MIN_OPPONENT_VELOCITY) {
                harassTarget.harassPosition = opponent->pos - Vector2{DEFAULT_HARASS_DISTANCE, 0};
            } else {
                Vector2 futureOpponentPos = opponent->pos + opponent->vel * HARASSER_SECONDS_AHEAD;

                //TODO: make sure projection is actually in front of opponent
                Vector2 projectionPoint = thisRobot->pos.project(opponent->pos, futureOpponentPos);
                if ((projectionPoint - thisRobot->pos).length() < STAND_STILL_DISTANCE) {
                    harassTarget.harassPosition = thisRobot->pos;
                } else {
                    harassTarget.harassPosition = projectionPoint;
                }
            }
            break;
        }
        case BLOCK_PASS: {
            //TODO: Make sure projection is actually between ball and opponent
            harassTarget.harassPosition = thisRobot->pos.project(ball->pos, opponent->pos);
            break;
        }
        case BALL: {
            harassTarget.harassPosition = ball->pos;
            break;
        }
    }
    return harassTarget;
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

Vector2 MidFieldCoach::standFree(const RobotPtr& thisRobot) {
    return thisRobot->pos;
}

MidFieldCoach::HarassType MidFieldCoach::getHarassType(const RobotPtr& thisRobot, const RobotPtr& opponent) {
    auto ball = world::world->getBall();

    // Check if the opponent is left of us (is being offensive)
    if (opponent->pos.x < thisRobot->pos.x) {

        // Check if ball is on our left
        if (ball->pos.x < thisRobot->pos.x) {
            return BALL;
            // Else, the ball is on our right
        } else {
            return BLOCK_PASS;
        }

        // Else, the opponent is on our right (being defensive or about to offend)
    } else {
        // Check if ball is on our left
        if (ball->pos.x < thisRobot->pos.x) {
            return HARASS;
            // Else, the ball is on our right
        } else {
            return BALL;
        }
    }
}

} //coach
} //ai
} //rtt